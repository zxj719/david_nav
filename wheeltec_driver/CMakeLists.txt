cmake_minimum_required(VERSION 3.8)
project(wheeltec_driver)

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)

add_library(wheeltec_protocol SHARED src/protocol.cpp)
target_compile_features(wheeltec_protocol PUBLIC cxx_std_17)
target_compile_options(wheeltec_protocol PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(wheeltec_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS wheeltec_protocol EXPORT export_wheeltec_protocol
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
add_library(wheeltec_transport SHARED src/serial_stream.cpp src/status_reader.cpp)
target_link_libraries(wheeltec_transport wheeltec_protocol)
ament_target_dependencies(wheeltec_transport serial)
target_compile_options(wheeltec_transport PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS wheeltec_transport EXPORT export_wheeltec_transport
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
  ament_add_gtest(test_status_reader test/test_status_reader.cpp)
  target_link_libraries(test_status_reader wheeltec_transport)
endif()

ament_export_targets(export_wheeltec_protocol HAS_LIBRARY_TARGET)
ament_export_targets(export_wheeltec_transport HAS_LIBRARY_TARGET)
ament_export_dependencies(serial)
ament_package()
