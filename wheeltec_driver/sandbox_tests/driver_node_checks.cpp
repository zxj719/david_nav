// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "support/fake_stream.hpp"
#include "wheeltec_driver/driver_node.hpp"

namespace wheeltec_driver
{

class DriverNodeTestAccess
{
public:
  static void cancelTimer(WheeltecDriverNode & node) {node.control_timer_->cancel();}
  static void control(WheeltecDriverNode & node) {node.control();}
  static void command(WheeltecDriverNode & node, double angular)
  {
    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->angular.z = angular;
    node.command(message);
  }
};

namespace
{

struct Trace
{
  std::vector<std::vector<uint8_t>> writes;
  std::string configured_port;
  uint32_t configured_baud{};
  unsigned int closes{};
};

class RecordingStream : public test::FakeStream
{
public:
  explicit RecordingStream(Trace & trace)
  : trace_(trace) {connected = false;}
  std::size_t write(const uint8_t * data, std::size_t size) override
  {
    const auto count = FakeStream::write(data, size);
    trace_.writes.emplace_back(data, data + count);
    return count;
  }
  void close() override
  {
    FakeStream::close();
    ++trace_.closes;
  }

private:
  Trace & trace_;
};

std::shared_ptr<WheeltecDriverNode> makeDriver(
  Trace & trace, RecordingStream ** captured,
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
{
  auto node = std::make_shared<WheeltecDriverNode>(
    options, [&trace, captured](const std::string & port, uint32_t baud) {
      trace.configured_port = port;
      trace.configured_baud = baud;
      auto stream = std::make_unique<RecordingStream>(trace);
      *captured = stream.get();
      return stream;
    });
  DriverNodeTestAccess::cancelTimer(*node);
  return node;
}

bool spinUntil(
  const std::shared_ptr<rclcpp::Node> & first, const std::shared_ptr<rclcpp::Node> & second,
  const std::function<bool()> & ready)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(first);
    rclcpp::spin_some(second);
    if (ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

class DriverNodeChecks : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const auto domain = std::getenv("ROS_DOMAIN_ID");
    if (domain == nullptr || std::string(domain) != "99") {
      throw std::runtime_error("driver checks require the guarded domain 99 launcher");
    }
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(DriverNodeChecks, NineLegacyParametersKeepInstalledDefaults)
{
  Trace trace;
  RecordingStream * stream{};
  auto node = makeDriver(trace, &stream);
  EXPECT_EQ(std::string(node->get_name()), "wheeltec_robot");
  EXPECT_EQ(node->get_parameter("usart_port_name").as_string(), "/dev/wheeltec_controller");
  EXPECT_EQ(node->get_parameter("serial_baud_rate").as_int(), 115200);
  EXPECT_EQ(node->get_parameter("odom_frame_id").as_string(), "odom_combined");
  EXPECT_EQ(node->get_parameter("robot_frame_id").as_string(), "base_footprint");
  EXPECT_EQ(node->get_parameter("gyro_frame_id").as_string(), "gyro_link");
  for (const auto name : {"odom_x_scale", "odom_y_scale", "odom_z_scale_positive",
      "odom_z_scale_negative"})
  {
    EXPECT_EQ(node->get_parameter(name).as_double(), 1.0);
  }
  EXPECT_EQ(stream->opens, 1u);
  EXPECT_EQ(trace.configured_baud, 115200u);
  EXPECT_TRUE(trace.writes.empty());
  // Direct callback -> memory stream only. No nonzero Twist is published.
  DriverNodeTestAccess::command(*node, 0.55);
  const auto expected = encodeCommand(0.0, 0.0, 0.55);
  ASSERT_EQ(trace.writes.size(), 1u);
  EXPECT_EQ(trace.writes[0], std::vector<uint8_t>(expected.begin(), expected.end()));
  node.reset();
  ASSERT_EQ(trace.writes.size(), 2u);
  const auto stop = encodeCommand(0.0, 0.0, 0.0);
  EXPECT_EQ(trace.writes.back(), std::vector<uint8_t>(stop.begin(), stop.end()));
  EXPECT_EQ(trace.closes, 1u);
}

TEST_F(DriverNodeChecks, OverridesAndRealZeroCommandCallbackUseOnlyMemoryStream)
{
  Trace trace;
  RecordingStream * stream{};
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        {"usart_port_name", "/memory-only"}, {"serial_baud_rate", 57600},
        {"odom_frame_id", "other_encoder"}, {"robot_frame_id", "other_axle"},
        {"gyro_frame_id", "other_gyro"}, {"odom_x_scale", 1.2}});
  options.arguments(
    {"--ros-args", "-r", "cmd_vel:=/driver_check/command",
      "-r", "odom:=/driver_check/odometry"});
  auto driver = makeDriver(trace, &stream, options);
  auto observer = std::make_shared<rclcpp::Node>("driver_check_observer");
  auto command = observer->create_publisher<geometry_msgs::msg::Twist>("/driver_check/command", 10);
  nav_msgs::msg::Odometry::SharedPtr received;
  auto odometry = observer->create_subscription<nav_msgs::msg::Odometry>(
    "/driver_check/odometry", 50, [&received](nav_msgs::msg::Odometry::SharedPtr message) {
      received = message;
    });
  ASSERT_TRUE(
    spinUntil(
      driver, observer, [&]() {
        return command->get_subscription_count() > 0 && odometry->get_publisher_count() > 0;
      }));
  command->publish(geometry_msgs::msg::Twist());  // The only published command is zero.
  ASSERT_TRUE(spinUntil(driver, observer, [&]() {return !trace.writes.empty();}));
  const StatusFrame golden{
    0x7B, 0x01, 0, 0, 0, 0, 0, 0, 0, 0x16, 0xFF, 0xF4,
    0x40, 0x02, 0, 0x01, 0xFF, 0xFF, 0, 0, 0x10, 0xF2, 0xC6, 0x7D};
  stream->incoming.insert(stream->incoming.end(), golden.begin(), golden.end());
  DriverNodeTestAccess::control(*driver);
  ASSERT_TRUE(spinUntil(driver, observer, [&]() {return received != nullptr;}));
  EXPECT_EQ(trace.configured_port, "/memory-only");
  EXPECT_EQ(trace.configured_baud, 57600u);
  EXPECT_EQ(received->header.frame_id, "other_encoder");
  EXPECT_EQ(received->child_frame_id, "other_axle");
  EXPECT_EQ(received->twist.twist.linear.x, 0.0);
  EXPECT_EQ(received->pose.pose.orientation.w, 1.0);
}

TEST_F(DriverNodeChecks, InvalidConfigurationIsRejectedBeforeFactoryCanOpenAnything)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"serial_baud_rate", -1}});
  bool factory_called = false;
  EXPECT_THROW(
    std::make_shared<WheeltecDriverNode>(
      options,
      [&factory_called](const std::string &, uint32_t) -> std::unique_ptr<ByteStream> {
        factory_called = true;
        return nullptr;
      }), std::invalid_argument);
  EXPECT_FALSE(factory_called);
}

}  // namespace
}  // namespace wheeltec_driver
