cmake_minimum_required(VERSION 3.5)
project(dis_tutorial4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

install(PROGRAMS
  scripts/map_goals.py
  scripts/transform_point.py
  scripts/detect_people.py
  scripts/LLM.py
  scripts/robot_commander.py
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/voice_capture.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_capture)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
