cmake_minimum_required(VERSION 3.8)
project(wheeltec_driver)

find_package(ament_cmake REQUIRED)

add_library(wheeltec_protocol SHARED src/protocol.cpp)
target_compile_features(wheeltec_protocol PUBLIC cxx_std_17)
target_compile_options(wheeltec_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(wheeltec_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS wheeltec_protocol EXPORT export_wheeltec_protocol
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
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
endif()

ament_export_targets(export_wheeltec_protocol HAS_LIBRARY_TARGET)
ament_package()
