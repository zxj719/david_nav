cmake_minimum_required(VERSION 3.8)
project(david_slam_gt)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python nodes installed as executables under lib/${PROJECT_NAME}
install(PROGRAMS
  david_slam_gt/odom_to_base_tf.py
  david_slam_gt/groundtruth_pose_publisher.py
  david_slam_gt/self_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Data: launch, config, rviz, urdf-less (reuse david_nav urdf), scripts
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/launch_slam_gt.sh
  scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
