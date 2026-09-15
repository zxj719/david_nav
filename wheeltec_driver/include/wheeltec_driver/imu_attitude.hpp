// Copyright 2026 David navigation contributors
// Adapted from Wheeltec Quaternion_Solution; upstream license remains unspecified (NOTICE).

#ifndef WHEELTEC_DRIVER__IMU_ATTITUDE_HPP_
#define WHEELTEC_DRIVER__IMU_ATTITUDE_HPP_

#include <array>

namespace wheeltec_driver
{

/// Instance-local legacy 20 Hz Mahony IMU attitude (Kp*2=1, Ki*2=0).
/// Retains the one-iteration inverse-square-root approximation, not an exact sqrt.
class ImuAttitude
{
public:
  /// Update from angular rates in rad/s and accelerations in m/s^2.
  /// Invalid/nonfinite numerical results return false without changing orientation.
  bool update(float gx, float gy, float gz, float ax, float ay, float az);
  /// Return x/y/z/w with the original float results promoted to doubles.
  std::array<double, 4> orientation() const;
  /// Restore the initial identity orientation.
  void reset();

private:
  std::array<float, 4> q_{1.0f, 0.0f, 0.0f, 0.0f};  // w/x/y/z internally, as in vendor code.
};

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__IMU_ATTITUDE_HPP_
