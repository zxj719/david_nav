// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "support/fake_stream.hpp"
#include "wheeltec_driver/driver_core.hpp"

namespace wheeltec_driver
{
namespace
{

StatusFrame status(int16_t vx = 0, int16_t vy = 0, int16_t wz = 0)
{
  StatusFrame frame{};
  frame.front() = kFrameHeader;
  frame.back() = kFrameTail;
  const std::array<int16_t, 3> values{vx, vy, wz};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto bits = static_cast<uint16_t>(values[axis]);
    frame[2 + axis * 2] = bits >> 8;
    frame[3 + axis * 2] = bits & 0xFF;
  }
  frame[22] = xorChecksum(frame.data(), 22);
  return frame;
}

void append(test::FakeStream * stream, const StatusFrame & frame)
{
  stream->incoming.insert(stream->incoming.end(), frame.begin(), frame.end());
}

std::vector<uint8_t> zeroFrame()
{
  const auto frame = encodeCommand(0.0, 0.0, 0.0);
  return {frame.begin(), frame.end()};
}

TEST(DriverCore, ConstructionDoesNotOpenOrWrite)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  EXPECT_EQ(stream.opens, 0u);
  EXPECT_TRUE(stream.writes.empty());
  EXPECT_FALSE(core.latest());
}

TEST(DriverCore, CommandsUseProtocolBytesAndDestructorStopsBeforeClose)
{
  test::FakeStream stream;
  {
    DriverCore core(stream, {1, 1, 1, 1}, 0.0);
    ASSERT_TRUE(core.command(0.0, 0.0, 0.55));
    const auto expected = encodeCommand(0.0, 0.0, 0.55);
    ASSERT_EQ(stream.writes.size(), 1u);
    EXPECT_EQ(stream.writes[0], std::vector<uint8_t>(expected.begin(), expected.end()));
  }
  ASSERT_EQ(stream.writes.size(), 2u);
  EXPECT_EQ(stream.writes.back(), zeroFrame());
  EXPECT_EQ(stream.closes, 1u);
}

TEST(DriverCore, ShutdownIsIdempotentAndBlocksFurtherWritesOrReconnect)
{
  test::FakeStream stream;
  {
    DriverCore core(stream, {1, 1, 1, 1}, 0.0);
    EXPECT_TRUE(core.shutdown());
    EXPECT_FALSE(core.shutdown());
    EXPECT_FALSE(core.command(0.1, 0.0, 0.0));
    EXPECT_FALSE(core.tick(1.0));
    EXPECT_FALSE(core.connect());
  }
  EXPECT_EQ(stream.writes.size(), 1u);
  EXPECT_EQ(stream.opens, 0u);
  EXPECT_EQ(stream.closes, 1u);
}

TEST(DriverCore, InvalidCommandAttemptsOneStopAndReportsRejection)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  EXPECT_FALSE(core.command(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  ASSERT_EQ(stream.writes.size(), 1u);
  EXPECT_EQ(stream.writes[0], zeroFrame());
  EXPECT_EQ(core.counters().rejected_commands, 1u);
}

TEST(DriverCore, ShortWriteAndIoFailureAreReportedWithoutThrowing)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  stream.write_limit = 5;
  EXPECT_FALSE(core.command(0.0, 0.0, 0.0));
  EXPECT_EQ(core.lastError(), "short serial command write");
  stream.fail_write = true;
  EXPECT_FALSE(core.command(0.0, 0.0, 0.0));
  EXPECT_EQ(core.counters().transmit_errors, 2u);
  EXPECT_FALSE(core.shutdown());
  EXPECT_FALSE(stream.connected);
}

TEST(DriverCore, IntegratesWithLegacyFloatPrecisionAndSignedYawScales)
{
  test::FakeStream stream;
  const std::array<float, 4> scales{1.02f, 0.97f, 1.1f, 0.9f};
  DriverCore core(stream, scales, 0.0);
  std::array<float, 3> expected{};
  double previous = 0;
  for (int tick = 1; tick <= 1000; ++tick) {
    const int16_t wz = tick % 2 == 0 ? 300 : -400;
    append(&stream, status(150, -20, wz));
    const double now = tick * 0.05;
    const auto sample = core.tick(now);
    ASSERT_TRUE(sample);
    const float vx = 0.15f * scales[0];
    const float vy = -0.02f * scales[1];
    const float yaw = decodeMilli(wz) * (wz >= 0 ? scales[2] : scales[3]);
    const double dt = now - previous;
    expected[0] += (vx * std::cos(expected[2]) - vy * std::sin(expected[2])) * dt;
    expected[1] += (vx * std::sin(expected[2]) + vy * std::cos(expected[2])) * dt;
    expected[2] += yaw * dt;
    EXPECT_EQ(sample->pose, expected);
    EXPECT_EQ(sample->velocity, (std::array<float, 3>{vx, vy, yaw}));
    previous = now;
  }
}

TEST(DriverCore, OnlyLastFrameInBatchIsIntegratedOverElapsedTime)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  append(&stream, status(100, 0, 0));
  append(&stream, status(200, 0, 0));
  const auto sample = core.tick(0.5);
  ASSERT_TRUE(sample);
  EXPECT_EQ(sample->pose[0], 0.1f);
  EXPECT_EQ(core.receiveCounters().valid_frames, 2u);
}

TEST(DriverCore, MissingFramesDoNotAdvanceSamplingTimeOrVoltageDivider)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  EXPECT_FALSE(core.tick(0.2));
  for (int count = 1; count <= 24; ++count) {
    append(&stream, status(100, 0, 0));
    const auto sample = core.tick(count * 0.5);
    ASSERT_TRUE(sample);
    EXPECT_EQ(sample->publish_voltage, count % 12 == 0);
  }
}

TEST(DriverCore, ResetSamplingOriginExcludesInitialConnectionDelay)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  core.resetSamplingTime(4.0);
  append(&stream, status(100, 0, 0));
  const auto sample = core.tick(4.5);
  ASSERT_TRUE(sample);
  EXPECT_EQ(sample->pose[0], 0.05f);
}

TEST(DriverCore, ClosedPortRetriesAtLegacyHundredTickCadence)
{
  test::FakeStream stream;
  stream.connected = false;
  stream.fail_open = true;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  for (int tick = 0; tick < 100; ++tick) {
    EXPECT_FALSE(core.tick(tick * 0.01));
  }
  EXPECT_EQ(stream.opens, 1u);
  stream.fail_open = false;
  EXPECT_FALSE(core.tick(1.0));
  EXPECT_EQ(stream.opens, 2u);
  EXPECT_TRUE(core.online());
  EXPECT_EQ(core.counters().successful_connections, 1u);
}

TEST(DriverCore, ReadErrorClosesAndReconnectionKeepsIntegratedPose)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  append(&stream, status(100, 0, 0));
  ASSERT_TRUE(core.tick(1.0));
  stream.fail_available = true;
  EXPECT_FALSE(core.tick(1.1));
  EXPECT_FALSE(core.online());
  stream.fail_available = false;
  EXPECT_FALSE(core.tick(1.2));
  append(&stream, status(100, 0, 0));
  const auto sample = core.tick(2.0);
  ASSERT_TRUE(sample);
  EXPECT_EQ(sample->pose[0], 0.2f);
}

TEST(DriverCore, InvalidCalibrationAndTimeFailWithoutOpeningDevice)
{
  test::FakeStream stream;
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(DriverCore(stream, {nan, 1, 1, 1}, 0.0), std::invalid_argument);
  EXPECT_THROW(DriverCore(stream, {1, 1, 1, 1}, nan), std::invalid_argument);
  EXPECT_EQ(stream.opens, 0u);
}

}  // namespace
}  // namespace wheeltec_driver
