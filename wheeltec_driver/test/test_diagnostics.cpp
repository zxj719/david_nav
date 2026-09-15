// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <limits>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "support/fake_stream.hpp"
#include "wheeltec_driver/diagnostics.hpp"

namespace wheeltec_driver
{
namespace
{

std::string value(const diagnostic_msgs::msg::DiagnosticArray & message, const std::string & key)
{
  for (const auto & entry : message.status.at(0).values) {
    if (entry.key == key) {
      return entry.value;
    }
  }
  throw std::runtime_error("missing diagnostic field: " + key);
}

TEST(Diagnostics, OfflineAndAwaitingFramesDoNotFabricateVoltageOrStopFlag)
{
  test::FakeStream stream;
  stream.connected = false;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  auto message = makeDiagnostics(core, 0.0, "memory", stamp);
  ASSERT_EQ(message.status.size(), 1u);
  EXPECT_EQ(message.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(message.status[0].hardware_id, "memory");
  EXPECT_EQ(message.header.stamp.sec, 42);
  EXPECT_EQ(value(message, "serial_online"), "false");
  EXPECT_EQ(value(message, "voltage"), "unavailable");
  EXPECT_EQ(value(message, "Flag_Stop"), "unavailable");
  EXPECT_EQ(value(message, "reconnect_attempts"), "0");
  EXPECT_TRUE(stream.writes.empty());
  EXPECT_EQ(stream.opens, 0u);
  ASSERT_TRUE(core.connect());
  message = makeDiagnostics(core, 0.0, "memory", stamp);
  EXPECT_EQ(message.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(value(message, "successful_connections"), "1");
  EXPECT_EQ(value(message, "reconnect_attempts"), "0");
}

TEST(Diagnostics, RecordedVoltageFlagAndLifetimeCountersAreReportedWithoutReinterpretation)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  ASSERT_TRUE(core.connect());
  const StatusFrame golden{
    0x7B, 0x01, 0, 0, 0, 0, 0, 0, 0, 0x16, 0xFF, 0xF4,
    0x40, 0x02, 0, 0x01, 0xFF, 0xFF, 0, 0, 0x10, 0xF2, 0xC6, 0x7D};
  auto bad = golden;
  bad[22] ^= 1;
  stream.incoming.insert(stream.incoming.end(), bad.begin(), bad.end());
  stream.incoming.insert(stream.incoming.end(), golden.begin(), golden.end());
  ASSERT_TRUE(core.tick(0.05));
  ASSERT_TRUE(core.connect());
  const builtin_interfaces::msg::Time stamp;
  const auto message = makeDiagnostics(core, 19.5, "memory", stamp);
  EXPECT_EQ(message.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(value(message, "Flag_Stop"), "1");
  EXPECT_NEAR(std::stod(value(message, "voltage")), 4.338, 1e-5);
  EXPECT_EQ(std::stod(value(message, "receive_hz")), 19.5);
  EXPECT_EQ(value(message, "valid_frames"), "1");
  EXPECT_EQ(value(message, "checksum_errors"), "1");
  EXPECT_EQ(value(message, "reconnect_attempts"), "1");
  EXPECT_TRUE(stream.writes.empty());
}

TEST(Diagnostics, InvalidReceiveRatesAreRejected)
{
  test::FakeStream stream;
  DriverCore core(stream, {1, 1, 1, 1}, 0.0);
  const builtin_interfaces::msg::Time stamp;
  EXPECT_THROW(makeDiagnostics(core, -1.0, "memory", stamp), std::invalid_argument);
  EXPECT_THROW(
    makeDiagnostics(core, std::numeric_limits<double>::infinity(), "memory", stamp),
    std::invalid_argument);
}

}  // namespace
}  // namespace wheeltec_driver
