// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "chassis_adapter/adapter_node.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "chassis_adapter/core.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace chassis_adapter
{
using Twist = geometry_msgs::msg::Twist;
using Diagnostics = diagnostic_msgs::msg::DiagnosticArray;

struct AdapterNode::Runtime
{
  Runtime(
    const AdapterConfig & settings, rclcpp::Clock::SharedPtr ros_clock,
    rclcpp::Publisher<Twist>::SharedPtr commands,
    rclcpp::Publisher<Diagnostics>::SharedPtr diagnostics,
    const std::string & node_name, rclcpp::Logger node_logger)
  : config(settings), clock(std::move(ros_clock)), state(config, clock->now().nanoseconds()),
    command_publisher(std::move(commands)), diagnostic_publisher(std::move(diagnostics)),
    name(node_name), logger(node_logger), diagnostic_time(std::chrono::steady_clock::now()) {}

  void receive(const Twist & message)
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (!stopped) {
      state.receive(
        message.linear.x, message.linear.y, message.angular.z,
        clock->now().nanoseconds());
    }
  }

  void tick()
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (stopped) {
      return;
    }
    const auto output = state.output(clock->now().nanoseconds());
    if (output) {
      Twist message;
      message.linear.x = output->first;
      message.angular.z = output->second;
      command_publisher->publish(message);
    }
  }

  void diagnose()
  {
    std::lock_guard<std::mutex> guard(mutex);
    if (stopped) {
      return;
    }
    const auto now = clock->now();
    const auto steady_now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(steady_now - diagnostic_time).count();
    const auto output = state.diagnosticOutput(now.nanoseconds());
    const bool stale = state.isStale(now.nanoseconds());
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = stale ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = stale ? "input stale: output released after stop" : "OK";
    auto append = [&status](const std::string & key, const auto & value) {
        std::ostringstream text;
        text.precision(17);
        text << value;
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = text.str();
        status.values.push_back(entry);
      };
    append("input_count", state.inputCount());
    append("timed_out", stale ? "True" : "False");
    append("output_linear_x", output.first);
    append("output_angular_z", output.second);
    append("max_linear_x", config.max_linear_x);
    append("max_angular_z", config.max_angular_z);
    append("min_linear_x", config.min_linear_x);
    append("min_angular_z", config.min_angular_z);
    append("min_stationary_angular_z", config.min_stationary_angular_z);
    append("input_hz", elapsed > 0.0 ? (state.inputCount() - diagnostic_count) / elapsed : 0.0);
    diagnostic_count = state.inputCount();
    diagnostic_time = steady_now;
    Diagnostics message;
    message.header.stamp = now;
    message.status.push_back(status);
    diagnostic_publisher->publish(message);
  }

  void stop()
  {
    // The shared runtime, rather than a raw Node pointer, also protects against
    // asynchronous context shutdown racing component unload or a timer callback.
    std::lock_guard<std::mutex> guard(mutex);
    if (stopped) {
      return;
    }
    stopped = true;
    for (int attempt = 0; attempt < 5; ++attempt) {
      try {
        command_publisher->publish(Twist());
      } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "Unable to publish stop command: %s", error.what());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  AdapterConfig config;
  rclcpp::Clock::SharedPtr clock;
  AdapterState state;
  rclcpp::Publisher<Twist>::SharedPtr command_publisher;
  rclcpp::Publisher<Diagnostics>::SharedPtr diagnostic_publisher;
  std::string name;
  rclcpp::Logger logger;
  std::chrono::steady_clock::time_point diagnostic_time;
  uint64_t diagnostic_count{0};
  std::mutex mutex;
  bool stopped{false};
};

AdapterNode::AdapterNode(const rclcpp::NodeOptions & options)
: Node("chassis_adapter", options)
{
  AdapterConfig config;
  config.input_cmd_vel_topic = declare_parameter("input_cmd_vel_topic", config.input_cmd_vel_topic);
  config.output_cmd_vel_topic = declare_parameter(
    "output_cmd_vel_topic", config.output_cmd_vel_topic);
  auto numeric = [this](const std::string & name, double fallback) {
      auto seed = rclcpp::ParameterValue(fallback);
      const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
      const auto found = overrides.find(name);
      if (found != overrides.end() && found->second.get_type() == rclcpp::PARAMETER_INTEGER) {
        seed = found->second;
      }
      const auto value = declare_parameter(name, seed);
      return value.get_type() == rclcpp::PARAMETER_INTEGER ?
             static_cast<double>(value.get<int64_t>()) : value.get<double>();
    };
  config.cmd_vel_timeout = numeric("cmd_vel_timeout", config.cmd_vel_timeout);
  config.min_linear_x = numeric("min_linear_x", config.min_linear_x);
  config.max_linear_x = numeric("max_linear_x", config.max_linear_x);
  config.min_angular_z = numeric("min_angular_z", config.min_angular_z);
  config.min_stationary_angular_z = numeric(
    "min_stationary_angular_z", config.min_stationary_angular_z);
  config.stationary_angular_threshold = numeric(
    "stationary_angular_threshold", config.stationary_angular_threshold);
  config.max_angular_z = numeric("max_angular_z", config.max_angular_z);
  config.publish_hz = numeric("publish_hz", config.publish_hz);
  config.validate();
  runtime_ = std::make_shared<Runtime>(
    config, get_clock(),
    create_publisher<Twist>(config.output_cmd_vel_topic, 10),
    create_publisher<Diagnostics>("/diagnostics", 10), get_fully_qualified_name(), get_logger());
  const std::weak_ptr<Runtime> weak = runtime_;
  subscription_ = create_subscription<Twist>(
    config.input_cmd_vel_topic, 10,
    [weak](Twist::ConstSharedPtr message) {
      if (const auto state = weak.lock()) {
        state->receive(*message);
      }
    });
  command_timer_ = rclcpp::create_timer(
    this, get_clock(),
    rclcpp::Duration::from_seconds(config.periodSeconds()), [weak]() {
      if (const auto state = weak.lock()) {
        state->tick();
      }
    });
  diagnostic_timer_ = rclcpp::create_timer(
    this, get_clock(),
    rclcpp::Duration::from_seconds(1.0), [weak]() {
      if (const auto state = weak.lock()) {
        state->diagnose();
      }
    });
  context_ = get_node_base_interface()->get_context();
  shutdown_handle_ = context_->add_pre_shutdown_callback(
    [weak]() {
      if (const auto state = weak.lock()) {
        state->stop();
      }
    });
  RCLCPP_INFO(
    get_logger(), "Command adapter ready: %s -> %s; timeout=%.2fs",
    config.input_cmd_vel_topic.c_str(), config.output_cmd_vel_topic.c_str(),
    config.cmd_vel_timeout);
}

AdapterNode::~AdapterNode()
{
  command_timer_->cancel();
  diagnostic_timer_->cancel();
  runtime_->stop();
  context_->remove_pre_shutdown_callback(shutdown_handle_);
}
}  // namespace chassis_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(chassis_adapter::AdapterNode)
