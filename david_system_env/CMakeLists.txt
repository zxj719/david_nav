cmake_minimum_required(VERSION 3.8)
project(david_system_env)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(environment_assets test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
