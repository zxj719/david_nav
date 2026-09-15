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
#include "support/log_capture.hpp"
#include "wheeltec_driver/driver_node.hpp"

namespace wheeltec_driver
{

class DriverNodeTestAccess
{
public:
  static void cancelTimer(WheeltecDriverNode & node)
  {
    node.control_timer_->cancel();
    node.report_timer_->cancel();
  }
  static void control(WheeltecDriverNode & node) {node.control();}
  static void report(WheeltecDriverNode & node) {node.reportTransport();}
  static bool hasImu(WheeltecDriverNode & node) {return node.imu_publisher_ != nullptr;}
  static bool hasVoltage(WheeltecDriverNode & node) {return node.voltage_publisher_ != nullptr;}
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

TEST_F(DriverNodeChecks, WatchdogUsesSteadyTimeWithoutClockOrIncomingStatus)
{
  Trace trace;
  RecordingStream * stream{};
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"cmd_timeout_sec", 0.02}, {"use_sim_time", true}});
  auto driver = makeDriver(trace, &stream, options);
  ASSERT_EQ(driver->now().nanoseconds(), 0);
  ASSERT_TRUE(trace.writes.empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  DriverNodeTestAccess::control(*driver);
  ASSERT_EQ(trace.writes.size(), 1u);
  const auto stop = encodeCommand(0.0, 0.0, 0.0);
  EXPECT_EQ(trace.writes.back(), std::vector<uint8_t>(stop.begin(), stop.end()));
  DriverNodeTestAccess::control(*driver);
  EXPECT_EQ(trace.writes.size(), 1u);
  DriverNodeTestAccess::command(*driver, 0.0);
  DriverNodeTestAccess::control(*driver);
  EXPECT_EQ(trace.writes.size(), 2u);
}

TEST_F(DriverNodeChecks, FrameLogsAreOffByDefaultAndOptInUsesDebug)
{
  for (const bool enabled : {false, true}) {
    Trace trace;
    RecordingStream * stream{};
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"log_tx_frames", enabled}});
    auto driver = makeDriver(trace, &stream, options);
    test::LogCapture capture;
    DriverNodeTestAccess::command(*driver, 0.55);  // Only an in-memory callback.
    EXPECT_EQ(capture.count("cmd_vel serial write", RCUTILS_LOG_SEVERITY_DEBUG), enabled ? 1u : 0u);
    EXPECT_EQ(capture.count("cmd_vel serial write", RCUTILS_LOG_SEVERITY_INFO), 0u);
  }
}

TEST_F(DriverNodeChecks, TransportErrorsAreAggregatedOncePerReportInterval)
{
  Trace trace;
  RecordingStream * stream{};
  auto driver = makeDriver(trace, &stream);
  test::LogCapture capture;
  StatusFrame bad{};
  bad.front() = kFrameHeader;
  bad.back() = kFrameTail;
  bad[22] = xorChecksum(bad.data(), 22) ^ 1;
  for (int count = 0; count < 2; ++count) {
    stream->incoming.insert(stream->incoming.end(), bad.begin(), bad.end());
    DriverNodeTestAccess::control(*driver);
  }
  stream->fail_available = true;
  DriverNodeTestAccess::control(*driver);
  stream->fail_available = false;
  DriverNodeTestAccess::control(*driver);  // Memory-only reconnect.
  EXPECT_EQ(capture.count("serial 1s summary", RCUTILS_LOG_SEVERITY_WARN), 0u);
  DriverNodeTestAccess::report(*driver);
  EXPECT_EQ(capture.count("checksum=2", RCUTILS_LOG_SEVERITY_WARN), 1u);
  EXPECT_EQ(capture.count("read_errors=1", RCUTILS_LOG_SEVERITY_WARN), 1u);
  EXPECT_EQ(capture.count("reconnect_attempts=1", RCUTILS_LOG_SEVERITY_WARN), 1u);
  DriverNodeTestAccess::report(*driver);
  EXPECT_EQ(capture.count("serial 1s summary", RCUTILS_LOG_SEVERITY_WARN), 1u);
}

TEST_F(DriverNodeChecks, DiagnosticsArePublishedOnTheReportTimerWithoutStatusInput)
{
  Trace trace;
  RecordingStream * stream{};
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "/diagnostics:=/driver_check/diagnostics"});
  auto driver = makeDriver(trace, &stream, options);
  auto observer = std::make_shared<rclcpp::Node>("diagnostic_check_observer");
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr received;
  auto subscription = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/driver_check/diagnostics", 10,
    [&received](diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {received = message;});
  ASSERT_TRUE(spinUntil(driver, observer, [&]() {return subscription->get_publisher_count() > 0;}));
  DriverNodeTestAccess::report(*driver);
  ASSERT_TRUE(spinUntil(driver, observer, [&]() {return received != nullptr;}));
  ASSERT_EQ(received->status.size(), 1u);
  EXPECT_EQ(received->status[0].name, "wheeltec_driver/serial");
  EXPECT_EQ(received->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(trace.writes.empty());
}

TEST_F(DriverNodeChecks, OptionalPublishersAreIndependentAndOdometryAlwaysRemains)
{
  for (int combination = 0; combination < 4; ++combination) {
    const bool imu_enabled = combination & 1;
    const bool voltage_enabled = combination & 2;
    Trace trace;
    RecordingStream * stream{};
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {{"publish_imu", imu_enabled}, {"publish_voltage", voltage_enabled}});
    const std::string prefix = "/publisher_check_" + std::to_string(combination);
    options.arguments(
      {"--ros-args", "-r", "imu:=" + prefix + "/imu",
        "-r", "PowerVoltage:=" + prefix + "/voltage", "-r", "odom:=" + prefix + "/odom"});
    auto driver = makeDriver(trace, &stream, options);
    EXPECT_EQ(DriverNodeTestAccess::hasImu(*driver), imu_enabled);
    EXPECT_EQ(DriverNodeTestAccess::hasVoltage(*driver), voltage_enabled);
    auto observer = std::make_shared<rclcpp::Node>("optional_publisher_observer");
    unsigned int imu_count = 0;
    unsigned int voltage_count = 0;
    unsigned int odom_count = 0;
    auto imu = observer->create_subscription<sensor_msgs::msg::Imu>(
      prefix + "/imu", 20, [&imu_count](sensor_msgs::msg::Imu::SharedPtr) {++imu_count;});
    auto voltage = observer->create_subscription<std_msgs::msg::Float32>(
      prefix + "/voltage", 10,
      [&voltage_count](std_msgs::msg::Float32::SharedPtr) {++voltage_count;});
    auto odom = observer->create_subscription<nav_msgs::msg::Odometry>(
      prefix + "/odom", 50, [&odom_count](nav_msgs::msg::Odometry::SharedPtr) {++odom_count;});
    ASSERT_TRUE(
      spinUntil(
        driver, observer, [&]() {
          return odom->get_publisher_count() > 0 &&
          (!imu_enabled || imu->get_publisher_count() > 0) &&
          (!voltage_enabled || voltage->get_publisher_count() > 0);
        }));
    const StatusFrame golden{
      0x7B, 0x01, 0, 0, 0, 0, 0, 0, 0, 0x16, 0xFF, 0xF4,
      0x40, 0x02, 0, 0x01, 0xFF, 0xFF, 0, 0, 0x10, 0xF2, 0xC6, 0x7D};
    for (int count = 0; count < 12; ++count) {
      stream->incoming.insert(stream->incoming.end(), golden.begin(), golden.end());
      DriverNodeTestAccess::control(*driver);
    }
    ASSERT_TRUE(
      spinUntil(
        driver, observer, [&]() {
          return odom_count == 12 && (!imu_enabled || imu_count == 12) &&
          (!voltage_enabled || voltage_count == 1);
        }));
    EXPECT_EQ(imu_count, imu_enabled ? 12u : 0u);
    EXPECT_EQ(voltage_count, voltage_enabled ? 1u : 0u);
    EXPECT_TRUE(trace.writes.empty());
  }
}

}  // namespace
}  // namespace wheeltec_driver
