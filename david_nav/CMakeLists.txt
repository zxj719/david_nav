cmake_minimum_required(VERSION 3.8)
project(david_nav)

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
find_package(std_srvs REQUIRED)
find_package(david_nav_msgs REQUIRED)

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

# Costmap and BT libraries are now exported by nav_plugins. The old plugin XML
# path remains installed as a compatibility descriptor, without registration.

install(DIRECTORY
  behavior_trees
  config
  scripts
  launch
  meshes
  rviz
  urdf
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
  david_nav/map_warmup_spin.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_warmup_spin
)

# Compatibility shim only; runtime is in nav_executor (symlink-install applies).
install(PROGRAMS
  david_nav/dock_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_navigator
)

install(PROGRAMS
  david_nav/cmd_vel_separator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_separator
)

install(PROGRAMS
  david_nav/monitor_nav2_frontier_cpu.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME monitor_nav2_frontier_cpu
)

# Compatibility shim only; implementation is nav_executor.app.cli, not this copy.
install(PROGRAMS
  david_nav/navigation_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_executor.py
)

install(PROGRAMS
  david_nav/semantic_pose_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semantic_pose_navigator.py
)

install(PROGRAMS
  david_nav/position_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME position_controller
)

install(PROGRAMS
  david_nav/cmd_vel_mux.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_mux
)

install(PROGRAMS
  david_nav/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_follower
)

install(PROGRAMS
  david_nav/dead_reckoning_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dead_reckoning_executor
)

install(PROGRAMS
  navigate_semantic_pose_with_debug_bag.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigate_semantic_pose_with_debug_bag.sh
)

install(TARGETS
  frontier_explorer_cpp
  DESTINATION lib/${PROJECT_NAME}
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

  ament_add_pytest_test(
    position_control_core_test
    test/test_position_control_core.py
  )

  # Refactor loop P0.4/P0.6: these unit tests existed but were never
  # registered, so colcon test did not run them.
  ament_add_pytest_test(
    nav2_action_bridge_test
    test/test_nav2_action_bridge.py
  )

  ament_add_pytest_test(
    dock_finish_test
    test/test_dock_finish.py
  )

  ament_add_pytest_test(
    dock_settle_test
    test/test_dock_settle.py
  )

  ament_add_pytest_test(
    grid_map_view_test
    test/test_grid_map_view.py
  )

  ament_add_pytest_test(
    nav2_frame_consistency_test
    test/test_nav2_frame_consistency.py
  )

  ament_add_pytest_test(
    odometry_tf_broadcaster_test
    test/test_odometry_tf_broadcaster.py
  )

  ament_add_pytest_test(
    waypoint_sparsifier_test
    test/test_waypoint_sparsifier.py
  )

  # test_dock_common.py is a script-style self check without test_* functions;
  # run it through a pytest wrapper so a failure is reported correctly.
  ament_add_pytest_test(
    dock_bench_common_test
    test/test_dock_bench_common_script.py
  )

  # Characterization (golden) tests of the executor wire contract, SPEC §6 P0.
  ament_add_pytest_test(
    executor_golden_test
    test/test_executor_golden.py
  )

  # alias 0: server module in precise_nav_bringup + precise docking of dock POIs (2026-09-15).
  ament_add_pytest_test(
    dock_routing_test
    test/test_dock_routing.py
  )

  ament_add_pytest_test(
    dock_navigator_action_test
    test/test_dock_navigator_action.py
  )

  ament_add_pytest_test(
    dock_approach_test
    test/test_dock_approach.py
  )

  ament_add_pytest_test(
    precise_nav_bringup_launch_test
    test/test_precise_nav_bringup_launch.py
  )

  ament_add_pytest_test(
    pure_contract_migration_test
    test/test_pure_contract_migration.py
  )

  ament_add_pytest_test(
    goal_resolution_migration_test
    test/test_goal_resolution_migration.py
  )

  ament_add_pytest_test(
    poi_repository_migration_test
    test/test_poi_repository_migration.py
  )

  ament_add_pytest_test(
    pose_source_migration_test
    test/test_pose_source_migration.py
  )

endif()

ament_package()
