// Copyright 2026 David navigation contributors
// Navigation-side node migration; vendor behavioral provenance is retained in NOTICE.

#include "wheeltec_driver/driver_node.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/parameter_map.hpp"
#include "wheeltec_driver/message_mapping.hpp"
#include "wheeltec_driver/serial_stream.hpp"

namespace wheeltec_driver
{
namespace
{

double steadySeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

WheeltecDriverNode::WheeltecDriverNode(const rclcpp::NodeOptions & options, StreamFactory factory)
: Node("wheeltec_robot", options)
{
  const auto defaults = rclcpp::parameter_map_from_yaml_file(
    ament_index_cpp::get_package_share_directory("wheeltec_driver") +
    "/config/wheeltec_driver.yaml");
  for (const auto & parameter : defaults.at("/wheeltec_robot")) {
    declare_parameter(parameter.get_name(), parameter.get_parameter_value());
  }
  const auto port = get_parameter("usart_port_name").as_string();
  const auto baud = get_parameter("serial_baud_rate").as_int();
  odom_frame_ = get_parameter("odom_frame_id").as_string();
  robot_frame_ = get_parameter("robot_frame_id").as_string();
  gyro_frame_ = get_parameter("gyro_frame_id").as_string();
  if (port.empty() || baud <= 0 || baud > std::numeric_limits<uint32_t>::max() ||
    odom_frame_.empty() || robot_frame_.empty() || gyro_frame_.empty())
  {
    throw std::invalid_argument("driver requires a port, valid baud and nonempty frame IDs");
  }
  const std::array<float, 4> scales{
    static_cast<float>(get_parameter("odom_x_scale").as_double()),
    static_cast<float>(get_parameter("odom_y_scale").as_double()),
    static_cast<float>(get_parameter("odom_z_scale_positive").as_double()),
    static_cast<float>(get_parameter("odom_z_scale_negative").as_double())};
  if (!factory) {
    factory = [](const std::string & path, uint32_t rate) {
        return std::make_unique<SerialStream>(path, rate);
      };
  }
  stream_ = factory(port, static_cast<uint32_t>(baud));
  if (!stream_) {
    throw std::invalid_argument("driver stream factory returned null");
  }
  core_ = std::make_unique<DriverCore>(
    *stream_, scales, now().seconds(), get_parameter("cmd_timeout_sec").as_double(),
    steadySeconds());
  voltage_publisher_ = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", 20);
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 100, std::bind(&WheeltecDriverNode::command, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Data ready");
  if (core_->connect()) {
    RCLCPP_INFO(get_logger(), "wheeltec_robot serial port opened");
  } else {
    RCLCPP_ERROR(
      get_logger(), "wheeltec_robot can not open serial port: %s",
      core_->lastError().c_str());
  }
  core_->resetSamplingTime(now().seconds());
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&WheeltecDriverNode::control, this));
}

WheeltecDriverNode::~WheeltecDriverNode()
{
  stop();
}

void WheeltecDriverNode::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (shutdown_requested_) {
    return;
  }
  shutdown_requested_ = true;
  if (control_timer_) {
    control_timer_->cancel();
  }
  if (core_ && core_->shutdown()) {
    RCLCPP_INFO(get_logger(), "driver shutdown: zero frame written; serial close attempted");
  } else {
    RCLCPP_WARN(get_logger(), "driver shutdown: zero frame not confirmed; serial close attempted");
  }
}

void registerShutdownHook(const std::shared_ptr<WheeltecDriverNode> & node)
{
  const std::weak_ptr<WheeltecDriverNode> weak = node;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    [weak]() {
      if (const auto locked = weak.lock()) {
        locked->stop();
      }
    });
}

void WheeltecDriverNode::command(const geometry_msgs::msg::Twist::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!core_->command(message->linear.x, message->linear.y, message->angular.z, steadySeconds())) {
    RCLCPP_ERROR(
      get_logger(), "Unable to send data through serial port: %s",
      core_->lastError().c_str());
    return;
  }
  if (std::fabs(message->linear.x) > 1e-4 || std::fabs(message->linear.y) > 1e-4 ||
    std::fabs(message->angular.z) > 1e-4)
  {
    const auto frame = encodeCommand(message->linear.x, message->linear.y, message->angular.z);
    RCLCPP_INFO(
      get_logger(), "cmd_vel serial write: vx=%.3f vy=%.3f wz=%.3f "
      "bytes=[%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]",
      message->linear.x, message->linear.y, message->angular.z,
      frame[0], frame[1], frame[2], frame[3], frame[4], frame[5],
      frame[6], frame[7], frame[8], frame[9], frame[10]);
  }
}

void WheeltecDriverNode::control()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto attempts = core_->counters().connection_attempts;
  const auto errors = core_->receiveCounters().io_errors;
  const auto sample = core_->tick(now().seconds());
  core_->watchdog(steadySeconds());  // Independent of ROS time and incoming status samples.
  if (core_->counters().connection_attempts != attempts) {
    if (core_->online()) {
      RCLCPP_INFO(get_logger(), "wheeltec_robot serial port reconnected");
    } else {
      RCLCPP_WARN(get_logger(), "serial port unavailable, retrying...");
    }
  }
  if (core_->receiveCounters().io_errors != errors) {
    RCLCPP_ERROR(get_logger(), "serial read failed: %s", core_->lastError().c_str());
  }
  if (!sample) {
    return;
  }
  const builtin_interfaces::msg::Time stamp = now();
  odom_publisher_->publish(makeOdometry(*sample, odom_frame_, robot_frame_, stamp));
  // Reuse the already-extracted compatibility helper; never modify or start alias 3.
  attitude_.update(
    sample->sensor.angular_velocity[0], sample->sensor.angular_velocity[1],
    sample->sensor.angular_velocity[2], sample->sensor.acceleration[0],
    sample->sensor.acceleration[1], sample->sensor.acceleration[2]);
  imu_publisher_->publish(makeImu(*sample, attitude_.orientation(), gyro_frame_, stamp));
  if (sample->publish_voltage) {
    std_msgs::msg::Float32 voltage;
    voltage.data = sample->sensor.voltage;
    voltage_publisher_->publish(voltage);
  }
}

}  // namespace wheeltec_driver
