// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CHASSIS_ADAPTER__ADAPTER_NODE_HPP_
#define CHASSIS_ADAPTER__ADAPTER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace chassis_adapter
{

/// Loadable equivalent of the Python adapter, with stop-before-context-shutdown.
class AdapterNode : public rclcpp::Node
{
public:
  /// Declare the legacy parameter contract and create configured endpoints.
  explicit AdapterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /// Stop on component unload as well as whole-context shutdown.
  ~AdapterNode() override;

private:
  struct Runtime;
  std::shared_ptr<Runtime> runtime_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::PreShutdownCallbackHandle shutdown_handle_;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace chassis_adapter

#endif  // CHASSIS_ADAPTER__ADAPTER_NODE_HPP_
