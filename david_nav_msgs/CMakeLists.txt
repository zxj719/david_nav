cmake_minimum_required(VERSION 3.8)
project(david_nav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveTo.action"
  "msg/NavEvent.msg"
  DEPENDENCIES geometry_msgs
)

install(FILES README.md CHANGELOG.rst DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(nav_event_contract test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
    TIMEOUT 60)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
