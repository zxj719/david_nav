cmake_minimum_required(VERSION 3.8)
project(david_hand_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetHandPose.srv"
  "srv/SetHandPreset.srv"
)

ament_package()
