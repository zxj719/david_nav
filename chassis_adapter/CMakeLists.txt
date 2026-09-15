cmake_minimum_required(VERSION 3.8)
project(chassis_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/chassis_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

add_library(adapter_core SHARED src/core.cpp)
target_compile_features(adapter_core PUBLIC cxx_std_17)
target_compile_options(adapter_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_library(adapter_component SHARED src/adapter_node.cpp)
target_link_libraries(adapter_component adapter_core)
target_compile_options(adapter_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(adapter_component rclcpp rclcpp_components geometry_msgs diagnostic_msgs)
rclcpp_components_register_nodes(adapter_component "chassis_adapter::AdapterNode")
add_executable(chassis_adapter_cpp src/main.cpp)
target_link_libraries(chassis_adapter_cpp adapter_component)
ament_target_dependencies(chassis_adapter_cpp rclcpp)
install(TARGETS adapter_core adapter_component EXPORT adapter_targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS chassis_adapter_cpp RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  add_executable(adapter_core_oracle test/core_oracle.cpp)
  target_link_libraries(adapter_core_oracle adapter_core)
  ament_add_pytest_test(adapter_python test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV ADAPTER_CORE_ORACLE=$<TARGET_FILE:adapter_core_oracle>
    TIMEOUT 120)
  ament_cpplint()
  ament_uncrustify()
endif()

ament_export_targets(adapter_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components geometry_msgs diagnostic_msgs)
ament_package()
