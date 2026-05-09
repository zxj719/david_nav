cmake_minimum_required(VERSION 3.8)
project(g1_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(
  frontier_goal_selector_lib
  src/frontier_goal_selector.cpp
)
target_compile_features(frontier_goal_selector_lib PUBLIC cxx_std_17)
target_include_directories(frontier_goal_selector_lib PUBLIC include)

add_executable(
  frontier_explorer_cpp
  src/frontier_explorer.cpp
  src/frontier_search.cpp
)
target_compile_features(frontier_explorer_cpp PUBLIC cxx_std_17)
target_include_directories(frontier_explorer_cpp PUBLIC include)
target_link_libraries(frontier_explorer_cpp frontier_goal_selector_lib)
ament_target_dependencies(
  frontier_explorer_cpp
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_msgs
  visualization_msgs
)

add_library(g1_robot_embedded_in_obstacle_condition_bt_node SHARED
  src/robot_embedded_in_obstacle_condition.cpp
)
target_compile_features(g1_robot_embedded_in_obstacle_condition_bt_node PUBLIC cxx_std_17)
target_compile_definitions(
  g1_robot_embedded_in_obstacle_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)
target_include_directories(
  g1_robot_embedded_in_obstacle_condition_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  g1_robot_embedded_in_obstacle_condition_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_costmap_2d
  nav2_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(g1_escape_obstacle_action_bt_node SHARED
  src/escape_obstacle_action.cpp
)
target_compile_features(g1_escape_obstacle_action_bt_node PUBLIC cxx_std_17)
target_compile_definitions(
  g1_escape_obstacle_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT
)
target_include_directories(
  g1_escape_obstacle_action_bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  g1_escape_obstacle_action_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_costmap_2d
  nav2_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(g1_nav_scan_priority_layer SHARED
  src/scan_priority_layer.cpp
  src/scan_priority_geometry.cpp
)
target_compile_features(g1_nav_scan_priority_layer PUBLIC cxx_std_17)
target_include_directories(
  g1_nav_scan_priority_layer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  g1_nav_scan_priority_layer
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(DIRECTORY
  behavior_trees
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(FILES
  costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  g1_nav/map_warmup_spin.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_warmup_spin
)

install(PROGRAMS
  g1_nav/monitor_nav2_frontier_cpu.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME monitor_nav2_frontier_cpu
)

install(PROGRAMS
  g1_nav/navigation_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_executor.py
)

install(TARGETS
  frontier_explorer_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  g1_robot_embedded_in_obstacle_condition_bt_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  g1_escape_obstacle_action_bt_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  g1_nav_scan_priority_layer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)

if(BUILD_TESTING)
  ament_add_gtest(
    frontier_completion_policy_test
    test/frontier_completion_policy_test.cpp
  )
  target_compile_features(frontier_completion_policy_test PUBLIC cxx_std_17)
  target_include_directories(frontier_completion_policy_test PUBLIC include)

  ament_add_gtest(
    frontier_navigation_result_policy_test
    test/frontier_navigation_result_policy_test.cpp
  )
  target_compile_features(frontier_navigation_result_policy_test PUBLIC cxx_std_17)
  target_include_directories(frontier_navigation_result_policy_test PUBLIC include)

  ament_add_gtest(
    frontier_blacklist_policy_test
    test/frontier_blacklist_policy_test.cpp
  )
  target_compile_features(frontier_blacklist_policy_test PUBLIC cxx_std_17)
  target_include_directories(frontier_blacklist_policy_test PUBLIC include)

  ament_add_gtest(
    frontier_goal_selector_test
    test/frontier_goal_selector_test.cpp
  )
  target_compile_features(frontier_goal_selector_test PUBLIC cxx_std_17)
  target_include_directories(frontier_goal_selector_test PUBLIC include)
  target_link_libraries(frontier_goal_selector_test frontier_goal_selector_lib)

  ament_add_gtest(
    frontier_search_test
    test/frontier_search_test.cpp
    src/frontier_search.cpp
  )
  target_compile_features(frontier_search_test PUBLIC cxx_std_17)
  target_include_directories(frontier_search_test PUBLIC include)

  ament_add_gtest(
    escape_obstacle_action_test
    test/escape_obstacle_action_test.cpp
    src/escape_obstacle_action.cpp
  )
  target_compile_features(escape_obstacle_action_test PUBLIC cxx_std_17)
  target_include_directories(escape_obstacle_action_test PUBLIC include)
  ament_target_dependencies(
    escape_obstacle_action_test
    behaviortree_cpp_v3
    geometry_msgs
    nav2_costmap_2d
    nav2_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
  )

  ament_add_pytest_test(
    realsense_nav2_integration_test
    test/test_realsense_nav2_integration.py
  )

  ament_add_pytest_test(
    headless_navigation_script_test
    test/test_headless_navigation_script.py
  )

  ament_add_pytest_test(
    navigation_executor_packaging_test
    test/test_navigation_executor_packaging.py
  )

  ament_add_pytest_test(
    navigation_executor_protocol_test
    test/test_navigation_executor_protocol.py
  )

  ament_add_pytest_test(
    navigation_executor_poi_test
    test/test_navigation_executor_poi.py
  )

  ament_add_pytest_test(
    navigation_executor_runtime_test
    test/test_navigation_executor_runtime.py
  )

  ament_add_gtest(
    scan_priority_geometry_test
    test/scan_priority_geometry_test.cpp
    src/scan_priority_geometry.cpp
  )
  target_compile_features(scan_priority_geometry_test PUBLIC cxx_std_17)
  target_include_directories(scan_priority_geometry_test PUBLIC include)

  ament_add_pytest_test(
    realsense_cpu_monitor_test
    test/test_realsense_cpu_monitor.py
  )
endif()

ament_package()
