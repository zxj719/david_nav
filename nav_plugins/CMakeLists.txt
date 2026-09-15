cmake_minimum_required(VERSION 3.8)
project(nav_plugins)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Library basenames are a compatibility interface used by existing Nav2 YAML.
add_library(david_robot_embedded_in_obstacle_condition_bt_node SHARED
  src/robot_embedded_in_obstacle_condition.cpp)
add_library(david_escape_obstacle_action_bt_node SHARED
  src/escape_obstacle_action.cpp)
set(bt_libraries
  david_robot_embedded_in_obstacle_condition_bt_node
  david_escape_obstacle_action_bt_node)
foreach(target IN LISTS bt_libraries)
  target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${target}
    behaviortree_cpp_v3 geometry_msgs nav2_behavior_tree nav2_costmap_2d
    nav2_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)
endforeach()

add_library(david_nav_scan_priority_layer SHARED
  src/scan_priority_layer.cpp src/scan_priority_geometry.cpp)
ament_target_dependencies(david_nav_scan_priority_layer
  geometry_msgs nav2_costmap_2d pluginlib rclcpp rclcpp_lifecycle sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
set(plugin_libraries ${bt_libraries} david_nav_scan_priority_layer)
foreach(target IN LISTS plugin_libraries)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endforeach()
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS ${plugin_libraries} EXPORT export_nav_plugins
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config behavior_trees DESTINATION share/${PROJECT_NAME})
install(FILES README.md costmap_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint()
  ament_uncrustify()
  ament_add_gtest(escape_obstacle_action_test test/escape_obstacle_action_test.cpp)
  target_link_libraries(escape_obstacle_action_test david_escape_obstacle_action_bt_node)
  ament_add_gtest(scan_priority_geometry_test test/scan_priority_geometry_test.cpp)
  target_link_libraries(scan_priority_geometry_test david_nav_scan_priority_layer)
  ament_add_gtest(robot_base_frame_test test/robot_base_frame_test.cpp)
  target_compile_features(robot_base_frame_test PUBLIC cxx_std_17)
  target_include_directories(robot_base_frame_test PUBLIC include)
  ament_add_gtest(recovery_resume_test test/recovery_resume_test.cpp)
  target_compile_features(recovery_resume_test PUBLIC cxx_std_17)
  ament_target_dependencies(recovery_resume_test behaviortree_cpp_v3)
  target_compile_definitions(recovery_resume_test PRIVATE
    LEGACY_BT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../david_nav/behavior_trees"
    TEMPLATE_BT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  # Deliberately not a CTest: node creation is allowed only via the guarded
  # sandbox launcher when no user stack is running (RUNBOOK S7).
  ament_add_gtest_executable(scan_priority_layer_checks
    sandbox_tests/scan_priority_layer_checks.cpp)
  target_link_libraries(scan_priority_layer_checks david_nav_scan_priority_layer)
endif()

ament_export_targets(export_nav_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 geometry_msgs nav2_behavior_tree
  nav2_costmap_2d nav2_msgs pluginlib rclcpp rclcpp_lifecycle sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
ament_package()
