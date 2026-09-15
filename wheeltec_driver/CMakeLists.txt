cmake_minimum_required(VERSION 3.8)
project(wheeltec_driver)

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(wheeltec_protocol SHARED src/protocol.cpp)
target_compile_features(wheeltec_protocol PUBLIC cxx_std_17)
target_compile_options(wheeltec_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(wheeltec_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS wheeltec_protocol EXPORT export_wheeltec_protocol
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
add_library(wheeltec_transport SHARED src/serial_stream.cpp src/status_reader.cpp src/driver_core.cpp)
target_link_libraries(wheeltec_transport wheeltec_protocol)
ament_target_dependencies(wheeltec_transport serial)
target_compile_options(wheeltec_transport PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS wheeltec_transport EXPORT export_wheeltec_transport
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
add_library(wheeltec_attitude SHARED src/imu_attitude.cpp)
target_compile_features(wheeltec_attitude PUBLIC cxx_std_17)
target_compile_options(wheeltec_attitude PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_include_directories(wheeltec_attitude PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS wheeltec_attitude EXPORT export_wheeltec_attitude
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
add_library(wheeltec_messages SHARED src/message_mapping.cpp)
target_link_libraries(wheeltec_messages wheeltec_transport)
ament_target_dependencies(wheeltec_messages builtin_interfaces nav_msgs sensor_msgs)
install(TARGETS wheeltec_messages EXPORT export_wheeltec_messages
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
add_library(wheeltec_node SHARED src/driver_node.cpp)
target_link_libraries(wheeltec_node wheeltec_messages wheeltec_attitude)
ament_target_dependencies(wheeltec_node ament_index_cpp geometry_msgs rclcpp std_msgs)
install(TARGETS wheeltec_node EXPORT export_wheeltec_node
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
# D9 renames the executable in a separate compatibility commit.
add_executable(wheeltec_robot_node src/wheeltec_robot_main.cpp)
target_link_libraries(wheeltec_robot_node wheeltec_node)
install(TARGETS wheeltec_robot_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md NOTICE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint()
  ament_uncrustify()
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol wheeltec_protocol)
  target_compile_definitions(test_protocol PRIVATE
    UPLINK_GOLDEN_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../third_party/wheeltec_original/golden/uplink_status_frames_2026-09-14.txt")
  ament_add_gtest(test_status_reader test/test_status_reader.cpp)
  target_link_libraries(test_status_reader wheeltec_transport)
  ament_add_gtest(test_driver_core test/test_driver_core.cpp)
  target_link_libraries(test_driver_core wheeltec_transport)
  ament_add_gtest(test_message_mapping test/test_message_mapping.cpp)
  target_link_libraries(test_message_mapping wheeltec_messages)
  # This executable creates ROS nodes, so only the guarded sandbox script runs it.
  ament_add_gtest_executable(driver_node_checks sandbox_tests/driver_node_checks.cpp)
  target_include_directories(driver_node_checks PRIVATE test)
  target_link_libraries(driver_node_checks wheeltec_node)
  add_executable(driver_signal_checks sandbox_tests/driver_signal_checks.cpp)
  target_include_directories(driver_signal_checks PRIVATE test)
  target_link_libraries(driver_signal_checks wheeltec_node)
  include(cmake/imu_reference.cmake)
  # Instrument both the new implementation and the sanitized vendor reference.
  # No ROS context or hardware is created by this pure numerical test.
  ament_add_gtest(test_imu_attitude
    test/test_imu_attitude.cpp src/imu_attitude.cpp "${imu_reference}")
  target_compile_features(test_imu_attitude PUBLIC cxx_std_17)
  target_include_directories(test_imu_attitude PRIVATE include test)
  target_compile_options(test_imu_attitude PRIVATE
    -ffp-contract=off -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_libraries(test_imu_attitude -fsanitize=address,undefined)
endif()

ament_export_targets(export_wheeltec_protocol HAS_LIBRARY_TARGET)
ament_export_targets(export_wheeltec_transport HAS_LIBRARY_TARGET)
ament_export_targets(export_wheeltec_attitude HAS_LIBRARY_TARGET)
ament_export_targets(export_wheeltec_messages HAS_LIBRARY_TARGET)
ament_export_targets(export_wheeltec_node HAS_LIBRARY_TARGET)
ament_export_dependencies(serial ament_index_cpp builtin_interfaces geometry_msgs
  nav_msgs rclcpp sensor_msgs std_msgs)
ament_package()
