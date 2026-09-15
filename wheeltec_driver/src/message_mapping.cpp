// Copyright 2026 David navigation contributors
// Message fields preserve the legacy Wheeltec driver contract (NOTICE).

#include "wheeltec_driver/message_mapping.hpp"

#include <cmath>

namespace wheeltec_driver
{
namespace
{

std::array<double, 36> odometryCovariance(bool stopped)
{
  std::array<double, 36> result{};
  result[0] = stopped ? 1e-9 : 1e-3;
  result[7] = 1e-3;
  result[8] = stopped ? 1e-9 : 0.0;
  result[14] = result[21] = result[28] = 1e6;
  result[35] = stopped ? 1e-9 : 1e3;
  return result;
}

}  // namespace

nav_msgs::msg::Odometry makeOdometry(
  const DriverSample & sample, const std::string & odom_frame, const std::string & robot_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame;
  message.child_frame_id = robot_frame;
  message.pose.pose.position.x = sample.pose[0];
  message.pose.pose.position.y = sample.pose[1];
  message.pose.pose.position.z = sample.pose[2];
  message.pose.pose.orientation.z = std::sin(sample.pose[2] * 0.5);
  message.pose.pose.orientation.w = std::cos(sample.pose[2] * 0.5);
  message.twist.twist.linear.x = sample.velocity[0];
  message.twist.twist.linear.y = sample.velocity[1];
  message.twist.twist.angular.z = sample.velocity[2];
  const bool stopped = sample.velocity[0] == 0 && sample.velocity[1] == 0 &&
    sample.velocity[2] == 0;
  message.pose.covariance = odometryCovariance(stopped);
  message.twist.covariance = message.pose.covariance;
  return message;
}

sensor_msgs::msg::Imu makeImu(
  const DriverSample & sample, const std::array<double, 4> & orientation,
  const std::string & gyro_frame, const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = gyro_frame;
  message.orientation.x = orientation[0];
  message.orientation.y = orientation[1];
  message.orientation.z = orientation[2];
  message.orientation.w = orientation[3];
  message.orientation_covariance[0] = message.orientation_covariance[4] = 1e6;
  message.orientation_covariance[8] = 1e-6;
  message.angular_velocity_covariance = message.orientation_covariance;
  message.angular_velocity.x = sample.sensor.angular_velocity[0];
  message.angular_velocity.y = sample.sensor.angular_velocity[1];
  message.angular_velocity.z = sample.sensor.angular_velocity[2];
  message.linear_acceleration.x = sample.sensor.acceleration[0];
  message.linear_acceleration.y = sample.sensor.acceleration[1];
  message.linear_acceleration.z = sample.sensor.acceleration[2];
  return message;
}

}  // namespace wheeltec_driver
