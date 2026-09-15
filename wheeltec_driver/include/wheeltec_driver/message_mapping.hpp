// Copyright 2026 David navigation contributors
// Message fields preserve the legacy Wheeltec driver contract (NOTICE).

#ifndef WHEELTEC_DRIVER__MESSAGE_MAPPING_HPP_
#define WHEELTEC_DRIVER__MESSAGE_MAPPING_HPP_

#include <array>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "wheeltec_driver/driver_core.hpp"

namespace wheeltec_driver
{

/// Map one core sample into the legacy odometry fields, covariance and yaw-as-z.
/// D6 height correction is deliberately not enabled during behavior-preserving migration.
nav_msgs::msg::Odometry makeOdometry(
  const DriverSample & sample, const std::string & odom_frame, const std::string & robot_frame,
  const builtin_interfaces::msg::Time & stamp);

/// Map optional chassis IMU data without touching the independent localization stack.
sensor_msgs::msg::Imu makeImu(
  const DriverSample & sample, const std::array<double, 4> & orientation,
  const std::string & gyro_frame, const builtin_interfaces::msg::Time & stamp);

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__MESSAGE_MAPPING_HPP_
