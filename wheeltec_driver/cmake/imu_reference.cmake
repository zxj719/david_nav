# Generate a test-only copy of the immutable vendor algorithm. Replace ONLY
# InvSqrt memory access; leave Quaternion_Solution and its volatile state intact.
set(vendor_imu "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/wheeltec_original/src/Quaternion_Solution.cpp")
file(READ "${vendor_imu}" vendor_text)
string(FIND "${vendor_text}" "float InvSqrt(float number)" inverse_start)
string(FIND "${vendor_text}" "volatile float twoKp" inverse_end)
if(inverse_start LESS 0 OR inverse_end LESS inverse_start)
  message(FATAL_ERROR "Vendor IMU reference changed; review extraction before updating tests")
endif()
string(SUBSTRING "${vendor_text}" 0 ${inverse_start} vendor_prefix)
string(REPLACE "#include \"Quaternion_Solution.h\"" "" vendor_prefix "${vendor_prefix}")
string(SUBSTRING "${vendor_text}" ${inverse_end} -1 vendor_algorithm)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/test/support/safe_inverse_reference.inc" safe_inverse)
set(imu_reference "${CMAKE_CURRENT_BINARY_DIR}/legacy_imu_reference.cpp")
file(WRITE "${imu_reference}"
  "// Generated test reference; vendor license remains unspecified.\n"
  "#include <cstdint>\n#include <cstring>\n#include \"support/legacy_imu_reference.hpp\"\n"
  "namespace legacy_reference {\nImu Mpu6050;\n"
  "${vendor_prefix}\n${safe_inverse}\n${vendor_algorithm}\n}\n")
