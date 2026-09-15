// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "support/legacy_imu_reference.hpp"
#include "wheeltec_driver/imu_attitude.hpp"

namespace wheeltec_driver
{
namespace
{

void resetReference()
{
  legacy_reference::q0 = 1.0f;
  legacy_reference::q1 = legacy_reference::q2 = legacy_reference::q3 = 0.0f;
  legacy_reference::integralFBx = 0.0f;
  legacy_reference::integralFBy = 0.0f;
  legacy_reference::integralFBz = 0.0f;
}

void compare(ImuAttitude * attitude, const std::array<float, 6> & input)
{
  ASSERT_TRUE(attitude->update(input[0], input[1], input[2], input[3], input[4], input[5]));
  legacy_reference::Quaternion_Solution(
    input[0], input[1], input[2], input[3], input[4], input[5]);
  const auto output = attitude->orientation();
  EXPECT_EQ(output[0], legacy_reference::Mpu6050.orientation.x);
  EXPECT_EQ(output[1], legacy_reference::Mpu6050.orientation.y);
  EXPECT_EQ(output[2], legacy_reference::Mpu6050.orientation.z);
  EXPECT_EQ(output[3], legacy_reference::Mpu6050.orientation.w);
}

TEST(ImuAttitude, RecordedRawFramesMatchSanitizedVendorReference)
{
  resetReference();
  ImuAttitude attitude;
  const std::array<std::array<float, 6>, 3> raw{{
    {1, -1, 0, 22, -12, 16386}, {3, -2, 0, -22, -4, 16288}, {-2, -1, 0, -8, -6, 16340}}};
  for (std::size_t tick = 0; tick < 3000; ++tick) {
    auto sample = raw[tick % raw.size()];
    for (std::size_t axis = 0; axis < 3; ++axis) {
      sample[axis] *= 0.00026644f;
      sample[axis + 3] /= 1671.84f;
    }
    compare(&attitude, sample);
  }
}

TEST(ImuAttitude, MotionAndZeroAccelerationMatchSanitizedVendorReference)
{
  resetReference();
  ImuAttitude attitude;
  for (int tick = 0; tick < 3000; ++tick) {
    const float phase = tick * 0.05f;
    const std::array<float, 6> sample{
      0.2f * std::sin(phase), 0.1f * std::cos(phase), -0.35f,
      tick % 2 == 0 ? 0.0f : 0.2f, 0.0f, tick % 2 == 0 ? 0.0f : 9.8f};
    compare(&attitude, sample);
  }
}

TEST(ImuAttitude, InstancesAreIndependentAndResettable)
{
  ImuAttitude first, second;
  const auto identity = second.orientation();
  ASSERT_TRUE(first.update(0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 9.8f));
  EXPECT_NE(first.orientation(), identity);
  EXPECT_EQ(second.orientation(), identity);
  first.reset();
  EXPECT_EQ(first.orientation(), identity);
}

TEST(ImuAttitude, InvalidInputsLeaveLastValidOrientationUntouched)
{
  ImuAttitude attitude;
  ASSERT_TRUE(attitude.update(0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 9.8f));
  const auto before = attitude.orientation();
  for (const auto invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), std::numeric_limits<float>::max()})
  {
    EXPECT_FALSE(attitude.update(invalid, 0.0f, 0.0f, 0.0f, 0.0f, 9.8f));
    EXPECT_FALSE(attitude.update(0.0f, 0.0f, 0.0f, invalid, 0.0f, 9.8f));
    EXPECT_EQ(attitude.orientation(), before);
  }
}

}  // namespace
}  // namespace wheeltec_driver
