// Copyright 2026 David navigation contributors
// Navigation-side node migration; vendor behavioral provenance is retained in NOTICE.

#ifndef WHEELTEC_DRIVER__DRIVER_NODE_HPP_
#define WHEELTEC_DRIVER__DRIVER_NODE_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "wheeltec_driver/driver_core.hpp"
#include "wheeltec_driver/imu_attitude.hpp"

namespace wheeltec_driver
{

using StreamFactory = std::function<std::unique_ptr<ByteStream>(const std::string &, uint32_t)>;

/// ROS adapter for the pure serial/odometry core, retaining the wheeltec_robot name.
class WheeltecDriverNode : public rclcpp::Node
{
public:
  /// Declare file-backed legacy defaults, apply normal ROS overrides and open the stream.
  /// Tests must supply a memory-only factory; the default factory accesses configured hardware.
  explicit WheeltecDriverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(), StreamFactory factory = {});
  ~WheeltecDriverNode() override;
  /// Stop/close once under the same lock as callbacks, with an observable result.
  void stop();

private:
  friend class DriverNodeTestAccess;
  void control();
  void reportTransport();
  void command(const geometry_msgs::msg::Twist::SharedPtr message);
  std::mutex mutex_;
  std::unique_ptr<ByteStream> stream_;
  std::unique_ptr<DriverCore> core_;  // Must be destroyed before stream_.
  ImuAttitude attitude_;
  std::string odom_frame_;
  std::string robot_frame_;
  std::string gyro_frame_;
  std::string hardware_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  ReceiveCounters reported_receive_{};
  DriverCounters reported_driver_{};
  bool log_tx_frames_{};
  double report_time_{};
  bool shutdown_requested_{};
};

/// Register a weak, context-bound callback after shared node construction.
void registerShutdownHook(const std::shared_ptr<WheeltecDriverNode> & node);

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__DRIVER_NODE_HPP_
