// Copyright 2026 David navigation contributors
// Test declarations only; generated implementation retains the vendor provenance.

#ifndef SUPPORT__LEGACY_IMU_REFERENCE_HPP_
#define SUPPORT__LEGACY_IMU_REFERENCE_HPP_

namespace legacy_reference
{

struct Orientation
{
  double x{}, y{}, z{}, w{};
};
struct Imu
{
  Orientation orientation;
};
extern Imu Mpu6050;
extern volatile float q0, q1, q2, q3;
extern volatile float integralFBx, integralFBy, integralFBz;
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az);

}  // namespace legacy_reference

#endif  // SUPPORT__LEGACY_IMU_REFERENCE_HPP_
