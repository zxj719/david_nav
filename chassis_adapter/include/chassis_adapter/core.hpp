// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CHASSIS_ADAPTER__CORE_HPP_
#define CHASSIS_ADAPTER__CORE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

namespace chassis_adapter
{

/// Legacy-compatible node defaults; robot values remain in profiles.
struct AdapterConfig
{
  std::string input_cmd_vel_topic{"cmd_vel_safe"};
  std::string output_cmd_vel_topic{"/cmd_vel"};
  double cmd_vel_timeout{2.0};
  double min_linear_x{0.20};
  double max_linear_x{0.20};
  double min_angular_z{0.0};
  double min_stationary_angular_z{0.35};
  double stationary_angular_threshold{0.025};
  double max_angular_z{0.50};
  double publish_hz{50.0};

  /// Reject empty topics, nonfinite settings and negative limits before I/O.
  void validate() const;
  /// Timer period, preserving the Python implementation's one-hertz floor.
  double periodSeconds() const;
};

using Command = std::pair<double, double>;

/// Clamp then lift, choosing the angular floor from the unmodified raw input.
Command shapeCommand(double x, double y, double yaw, const AdapterConfig & config);

/// Pure ROS-time state with a one-shot stale zero and no I/O or clock ownership.
class AdapterState
{
public:
  /// Initialize the legacy stale state. Time is expressed in integer nanoseconds.
  AdapterState(const AdapterConfig & config, int64_t now);
  /// Store shaped forward/yaw values and refresh input age without publishing.
  void receive(double x, double y, double yaw, int64_t now);
  /// Strict greater-than timeout; nonpositive timeout disables expiration.
  bool isStale(int64_t now) const;
  /// Return current output, one stale zero, or nullopt when ownership is released.
  std::optional<Command> output(int64_t now);
  /// Diagnostic output does not consume the one-shot stop transition.
  Command diagnosticOutput(int64_t now) const;
  /// Total received commands, including zeros.
  uint64_t inputCount() const;

private:
  AdapterConfig config_;
  Command latest_{0.0, 0.0};
  int64_t last_input_;
  uint64_t input_count_{0};
  bool stale_stop_sent_{true};
};

}  // namespace chassis_adapter

#endif  // CHASSIS_ADAPTER__CORE_HPP_
