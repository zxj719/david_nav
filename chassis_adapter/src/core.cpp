// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "chassis_adapter/core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace chassis_adapter
{
namespace
{
double clamp(double value, double limit)
{
  return std::max(std::min(value, std::abs(limit)), -std::abs(limit));
}

double lift(double value, double minimum)
{
  minimum = std::abs(minimum);
  if (value == 0.0 || minimum <= 0.0 || std::abs(value) >= minimum) {
    return value;
  }
  return value > 0.0 ? minimum : -minimum;
}
}  // namespace

void AdapterConfig::validate() const
{
  for (const auto & topic : {input_cmd_vel_topic, output_cmd_vel_topic}) {
    if (topic.find_first_not_of(" \t\n\r\f\v") == std::string::npos) {
      throw std::invalid_argument("command topics must be nonempty");
    }
  }
  for (double value : {cmd_vel_timeout, publish_hz}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("timeout and frequency must be finite");
    }
  }
  for (double value : {min_linear_x, max_linear_x, min_angular_z,
      min_stationary_angular_z, stationary_angular_threshold, max_angular_z})
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("limits must be finite and nonnegative");
    }
  }
}

double AdapterConfig::periodSeconds() const
{
  return 1.0 / std::max(publish_hz, 1.0);
}

Command shapeCommand(double x, double y, double yaw, const AdapterConfig & config)
{
  double minimum = config.min_angular_z;
  if (yaw == 0.0) {
    minimum = 0.0;
  } else if (!(std::abs(x) > 0.0 || std::abs(y) > 0.0)) {
    if (std::abs(yaw) >= std::abs(config.stationary_angular_threshold)) {
      minimum = config.min_stationary_angular_z;
    }
  }
  return {
    lift(clamp(x, config.max_linear_x), std::min(config.min_linear_x, config.max_linear_x)),
    lift(clamp(yaw, config.max_angular_z), std::min(minimum, config.max_angular_z))};
}

AdapterState::AdapterState(const AdapterConfig & config, int64_t now)
: config_(config), last_input_(now)
{
  config_.validate();
  constexpr int64_t kInitialAge = 3600LL * 1000000000LL;
  // ROS time is normally nonnegative; reject the unrepresentable corner rather
  // than invoking signed overflow when constructing the Python stale offset.
  if (now < std::numeric_limits<int64_t>::min() + kInitialAge) {
    throw std::invalid_argument("initial clock cannot represent the stale offset");
  }
  last_input_ -= kInitialAge;
}

void AdapterState::receive(double x, double y, double yaw, int64_t now)
{
  latest_ = shapeCommand(x, y, yaw, config_);
  last_input_ = now;
  ++input_count_;
  stale_stop_sent_ = false;
}

bool AdapterState::isStale(int64_t now) const
{
  const auto elapsed = static_cast<double>(static_cast<long double>(now) - last_input_);
  return config_.cmd_vel_timeout > 0.0 && elapsed / 1e9 > config_.cmd_vel_timeout;
}

std::optional<Command> AdapterState::output(int64_t now)
{
  if (isStale(now)) {
    if (!stale_stop_sent_) {
      stale_stop_sent_ = true;
      return Command{0.0, 0.0};
    }
    return std::nullopt;
  }
  return latest_;
}

Command AdapterState::diagnosticOutput(int64_t now) const
{
  return isStale(now) ? Command{0.0, 0.0} : latest_;
}

uint64_t AdapterState::inputCount() const
{
  return input_count_;
}
}  // namespace chassis_adapter
