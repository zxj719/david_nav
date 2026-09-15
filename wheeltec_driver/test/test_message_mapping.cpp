// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "wheeltec_driver/message_mapping.hpp"

namespace wheeltec_driver
{
namespace
{

TEST(MessageMapping, FramesPoseTwistAndLegacyHeightArePreserved)
{
  DriverSample sample;
  sample.pose = {1.25f, -0.5f, 0.4f};
  sample.velocity = {0.1f, -0.02f, 0.3f};
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 17;
  const auto message = makeOdometry(sample, "encoder_test", "axle_test", stamp);
  EXPECT_EQ(message.header.frame_id, "encoder_test");
  EXPECT_EQ(message.child_frame_id, "axle_test");
  EXPECT_EQ(message.header.stamp.sec, 17);
  EXPECT_EQ(message.pose.pose.position.x, 1.25f);
  EXPECT_EQ(message.pose.pose.position.y, -0.5f);
  EXPECT_EQ(message.pose.pose.position.z, 0.4f);  // D6 intentionally not enabled.
  EXPECT_EQ(message.pose.pose.orientation.z, std::sin(0.4f * 0.5));
  EXPECT_EQ(message.pose.pose.orientation.w, std::cos(0.4f * 0.5));
  EXPECT_EQ(message.twist.twist.linear.x, 0.1f);
  EXPECT_EQ(message.twist.twist.linear.y, -0.02f);
  EXPECT_EQ(message.twist.twist.angular.z, 0.3f);
}

TEST(MessageMapping, AllCovarianceEntriesMatchTheLegacyArrays)
{
  const std::array<double, 36> moving{
    1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3};
  const std::array<double, 36> stopped{
    1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9};
  DriverSample sample;
  const builtin_interfaces::msg::Time stamp;
  auto message = makeOdometry(sample, "encoder_test", "axle_test", stamp);
  EXPECT_EQ(message.pose.covariance, stopped);
  EXPECT_EQ(message.twist.covariance, stopped);
  sample.velocity[2] = 1e-8f;
  message = makeOdometry(sample, "encoder_test", "axle_test", stamp);
  EXPECT_EQ(message.pose.covariance, moving);
  EXPECT_EQ(message.twist.covariance, moving);
}

TEST(MessageMapping, OptionalChassisImuMappingDoesNotRequireLocalization)
{
  DriverSample sample;
  sample.sensor.acceleration = {1.0, 2.0, 3.0};
  sample.sensor.angular_velocity = {0.1, 0.2, 0.3};
  const builtin_interfaces::msg::Time stamp;
  const auto message = makeImu(sample, {0.0, 0.0, 0.1, 0.99}, "gyro_test", stamp);
  EXPECT_EQ(message.header.frame_id, "gyro_test");
  EXPECT_EQ(message.orientation.z, 0.1);
  EXPECT_EQ(message.orientation.w, 0.99);
  EXPECT_EQ(message.linear_acceleration.z, 3.0);
  EXPECT_EQ(message.angular_velocity.y, 0.2);
  EXPECT_EQ(message.orientation_covariance[0], 1e6);
  EXPECT_EQ(message.orientation_covariance[8], 1e-6);
  EXPECT_EQ(message.angular_velocity_covariance, message.orientation_covariance);
  EXPECT_EQ(message.linear_acceleration_covariance, (std::array<double, 9>{}));
}

}  // namespace
}  // namespace wheeltec_driver
