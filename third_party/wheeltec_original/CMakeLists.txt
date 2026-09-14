cmake_minimum_required(VERSION 3.8)
project(turn_on_wheeltec_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 保留原来的编译选项
add_compile_options(-g -std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(wheeltec_robot_node
  src/wheeltec_robot.cpp
  src/Quaternion_Solution.cpp
)

ament_target_dependencies(wheeltec_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  serial
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(twist_transform
  src/twist_transform.cpp
)

ament_target_dependencies(twist_transform
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  wheeltec_robot_node
  twist_transform
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keyboard_teleop_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  config
  map
  params_costmap_car
  params_costmap_common
  params_nav_common
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()