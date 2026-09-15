// Copyright 2026 David navigation contributors
// Adapted from Wheeltec Quaternion_Solution; upstream license remains unspecified (NOTICE).

#include "wheeltec_driver/imu_attitude.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace wheeltec_driver
{
namespace
{

float inverseSqrt(float number)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "legacy IMU requires 32-bit float");
  const float half = number * 0.5F;
  float estimate = number;
  uint32_t bits;
  // uint32_t and memcpy replace the vendor's long*/float* aliasing. On aarch64,
  // long is eight bytes and the old load overreads its four-byte float object.
  std::memcpy(&bits, &estimate, sizeof(bits));
  bits = 0x5f375a86U - (bits >> 1);
  std::memcpy(&estimate, &bits, sizeof(estimate));
  return estimate * (1.5F - (half * estimate * estimate));
}

}  // namespace

bool ImuAttitude::update(float gx, float gy, float gz, float ax, float ay, float az)
{
  const std::array<float, 6> inputs{gx, gy, gz, ax, ay, az};
  if (!std::all_of(inputs.begin(), inputs.end(), [](float value) {return std::isfinite(value);})) {
    return false;
  }
  auto next = q_;
  auto & q0 = next[0];
  auto & q1 = next[1];
  auto & q2 = next[2];
  auto & q3 = next[3];
  if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
    const auto norm = ax * ax + ay * ay + az * az;
    if (!std::isfinite(norm) || norm <= 0.0f) {
      return false;
    }
    const float reciprocal = inverseSqrt(norm);
    ax *= reciprocal;
    ay *= reciprocal;
    az *= reciprocal;
    const float halfvx = q1 * q3 - q0 * q2;
    const float halfvy = q0 * q1 + q2 * q3;
    const float halfvz = q0 * q0 - 0.5f + q3 * q3;
    const float halfex = ay * halfvz - az * halfvy;
    const float halfey = az * halfvx - ax * halfvz;
    const float halfez = ax * halfvy - ay * halfvx;
    // The old node never changes its global twoKi=0 or twoKp=1 values.
    gx += 1.0f * halfex;
    gy += 1.0f * halfey;
    gz += 1.0f * halfez;
  }
  constexpr float kHalfStep = 0.5f * (1.0f / 20.0f);
  gx *= kHalfStep;
  gy *= kHalfStep;
  gz *= kHalfStep;
  const float qa = q0;
  const float qb = q1;
  const float qc = q2;
  q0 += -qb * gx - qc * gy - q3 * gz;
  q1 += qa * gx + qc * gz - q3 * gy;
  q2 += qa * gy - qb * gz + q3 * gx;
  q3 += qa * gz + qb * gy - qc * gx;
  const auto norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (!std::isfinite(norm) || norm <= 0.0f) {
    return false;
  }
  const float reciprocal = inverseSqrt(norm);
  for (auto & value : next) {
    value *= reciprocal;
  }
  if (!std::all_of(next.begin(), next.end(), [](float value) {return std::isfinite(value);})) {
    return false;
  }
  q_ = next;
  return true;
}

std::array<double, 4> ImuAttitude::orientation() const
{
  return {q_[1], q_[2], q_[3], q_[0]};
}

void ImuAttitude::reset()
{
  q_ = {1.0f, 0.0f, 0.0f, 0.0f};
}

}  // namespace wheeltec_driver
