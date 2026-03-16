#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import yaml
import json

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

from ultralytics import YOLO

# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()
		self.scan = None

		self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		self.marker_array_pub = self.create_publisher(MarkerArray, "/people_marker_array", QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		# we will save face positions in the map coordinates:
		self.face_position_in_map_coordinates = []
		# minimum distance (meters) between two detections to consider them different people
		self.dedup_distance = 1.5

		# map info for saving detections to PGM
		self.map_yaml_path = '/home/erik/rins/maps.yaml'
		self.map_pgm_path = '/home/erik/rins/maps.pgm'
		self.detections_json_path = '/home/erik/rins/people_detections.json'

		# load previously saved detections if available
		self.load_detections()

		# we need TF2 Listener, which will store the /tf and /tf_static topic messages into buffer which we will use to make transformation 
		# between camera and map frames:
		self.tf_buf = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
		self.create_timer(2.0, self.log_face_positions)

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def rgb_callback(self, data):

		self.faces = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.get_logger().info(f"Running inference on image...")

			# run inference
			res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

			# iterate over results
			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0: # skip if empty
					continue

				self.get_logger().info(f"Person has been detected!")

				bbox = bbox[0]

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2)

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))

			cv2.imshow("image", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)

	def pointcloud_callback(self, data):

		# get point cloud attributes
		height = data.height
		width = data.width
		point_step = data.point_step
		row_step = data.row_step

		# iterate over face coordinates
		for x,y in self.faces:

			# get 3-channel representation of the poitn cloud in numpy format
			a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))   # reads from PointCloud2 message (topic /oakd/rgb/preview/depth/points to which our node is subscribed) and returns numpy array with x,y,z coordinates of each point
			a = a.reshape((height,width,3))

			# read center coordinates
			d = a[y,x,:]

			# skip invalid (NaN) depth readings
			if np.isnan(d[0]) or np.isnan(d[1]) or np.isnan(d[2]):
				self.get_logger().warn("Skipping detection with NaN depth")
				continue

			# we know that the topic for point for face is /oakd/rgb/preview/depth/points
			# by running ros2 topic echo --once /oakd/rgb/preview/depth/points we see that datapoints come in coordinates of frame oakd_rgb_camera_optical_frame
			# we want to get map coordinates of the face from the oakd_rgb_camera_optical_frame coordinates
			# thus we transform point from oakd_rgb_camera_optical_frame to map frame using TF2 
			try:
				point_in_oakd_rgb_camera_optical_frame = PointStamped()
				point_in_oakd_rgb_camera_optical_frame.header.frame_id = "oakd_rgb_camera_optical_frame"
				point_in_oakd_rgb_camera_optical_frame.header.stamp = rclpy.time.Time().to_msg()  # use Time() (latest available) instead of exact message stamp to avoid extrapolation-into-future errors
				point_in_oakd_rgb_camera_optical_frame.point.x = float(d[0])
				point_in_oakd_rgb_camera_optical_frame.point.y = float(d[1])
				point_in_oakd_rgb_camera_optical_frame.point.z = float(d[2])

				point_in_map = self.tf_buf.transform(point_in_oakd_rgb_camera_optical_frame, "map", timeout=Duration(seconds=0.5))

				map_x = point_in_map.point.x
				map_y = point_in_map.point.y
				map_z = point_in_map.point.z

				# we shift the detection point away from the wall toward the robot by looking up the robot's position via the base_link -> map transform
				offset_distance = 0.3  # meters to push the point away from the wall
				try:
					# get map coordinates of robot:
					trans = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.5))
					robot_x = trans.transform.translation.x
					robot_y = trans.transform.translation.y

					# the vector from robot to detected face approximates the camera's
					# line of sight, which is roughly perpendicular to the wall
					# we offset the detection back toward the robot along this vector
					dx = robot_x - map_x
					dy = robot_y - map_y
					dist = np.sqrt(dx**2 + dy**2)

					if dist > 0.01:
						map_x += (dx / dist) * offset_distance
						map_y += (dy / dist) * offset_distance

				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
					self.get_logger().warn(f"Could not look up base_link→map transform for offset: {e}")

				# deduplicate: only add if no existing detection is within self.dedup_distance
				is_new = True
				matched_idx = -1
				for i, (ex, ey, ez) in enumerate(self.face_position_in_map_coordinates):
					dist = np.sqrt((map_x - ex)**2 + (map_y - ey)**2)
					if dist < self.dedup_distance:
						is_new = False
						matched_idx = i
						break

				if is_new:
					self.face_position_in_map_coordinates.append((map_x, map_y, map_z))
					self.get_logger().info(f"NEW face in map coordinates: ({map_x:.2f}, {map_y:.2f}, {map_z:.2f})")
				else:
					# update stored position with running average to improve accuracy
					ex, ey, ez = self.face_position_in_map_coordinates[matched_idx]
					avg_x = (ex + map_x) / 2.0
					avg_y = (ey + map_y) / 2.0
					avg_z = (ez + map_z) / 2.0
					self.face_position_in_map_coordinates[matched_idx] = (avg_x, avg_y, avg_z)

			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				self.get_logger().warn(f"Could not transform to map frame: {e}")

		# re-publish all known detections as a MarkerArray so they all stay visible
		self.publish_all_markers()

	def publish_all_markers(self):
		"""Publish a MarkerArray with all stored face positions in the map frame."""
		marker_array = MarkerArray()
		for i, (mx, my, mz) in enumerate(self.face_position_in_map_coordinates):
			marker = Marker()
			marker.header.frame_id = "map"
			marker.header.stamp = self.get_clock().now().to_msg()
			marker.ns = "people_detections"
			marker.id = i
			marker.type = 2

			scale = 0.1
			marker.scale.x = scale
			marker.scale.y = scale
			marker.scale.z = scale

			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 1.0
			marker.color.a = 1.0

			marker.pose.position.x = mx
			marker.pose.position.y = my
			marker.pose.position.z = mz

			marker_array.markers.append(marker)

		self.marker_array_pub.publish(marker_array)

	def load_detections(self):
		"""Load previously saved detections from JSON file."""
		try:
			with open(self.detections_json_path, 'r') as f:
				data = json.load(f)
			self.face_position_in_map_coordinates = [tuple(d) for d in data]
			self.get_logger().info(f"Loaded {len(self.face_position_in_map_coordinates)} previous detections from {self.detections_json_path}")
		except FileNotFoundError:
			self.get_logger().info("No previous detections file found, starting fresh.")
		except Exception as e:
			self.get_logger().warn(f"Could not load detections: {e}")

	def save_detections_to_json(self):
		"""Save current detections to a JSON file for persistence."""
		try:
			with open(self.detections_json_path, 'w') as f:
				json.dump(self.face_position_in_map_coordinates, f, indent=2)
			self.get_logger().info(f"Saved {len(self.face_position_in_map_coordinates)} detections to {self.detections_json_path}")
		except Exception as e:
			self.get_logger().error(f"Failed to save detections to JSON: {e}")

	def log_face_positions(self):
		if self.face_position_in_map_coordinates:
			self.get_logger().info(f"Detected {len(self.face_position_in_map_coordinates)} unique people in map coordinates:")
			for i, (x, y, z) in enumerate(self.face_position_in_map_coordinates):
				self.get_logger().info(f"  Person {i+1}: ({x:.2f}, {y:.2f}, {z:.2f})")
		else:
			self.get_logger().info("No faces detected yet")

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.save_detections_to_json()
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()