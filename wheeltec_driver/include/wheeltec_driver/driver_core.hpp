// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WHEELTEC_DRIVER__DRIVER_CORE_HPP_
#define WHEELTEC_DRIVER__DRIVER_CORE_HPP_

#include <array>
#include <optional>
#include <string>

#include "wheeltec_driver/byte_stream.hpp"
#include "wheeltec_driver/status_reader.hpp"

namespace wheeltec_driver
{

/// One accepted control tick, independent of ROS publication and localization.
struct DriverSample
{
  ControllerState sensor;
  std::array<float, 3> velocity{};  ///< Scaled forward/lateral/yaw velocities.
  std::array<float, 3> pose{};  ///< Integrated x/y/yaw; legacy float accumulation.
  bool publish_voltage{};  ///< Preserve the legacy every-twelfth-sample divider.
};

/// Lifetime counters; initial open is included in connection_attempts.
struct DriverCounters
{
  uint64_t connection_attempts{};
  uint64_t successful_connections{};
  uint64_t transmitted_frames{};
  uint64_t transmit_errors{};
  uint64_t rejected_commands{};
};

/// Serial control and odometry, with all time and byte transport supplied by callers.
/// The owning node serializes access; stream must outlive this core.
class DriverCore
{
public:
  /// Save calibration and start time without opening a device.
  /// Scales are x/y/positive-yaw/negative-yaw, each rounded to legacy float precision.
  /// @throws std::invalid_argument if calibration or initial time is nonfinite.
  DriverCore(ByteStream & stream, const std::array<float, 4> & scales, double initial_time);
  /// Best-effort zero command followed by close, as in the old node destructor.
  ~DriverCore();
  DriverCore(const DriverCore &) = delete;
  DriverCore & operator=(const DriverCore &) = delete;

  /// Attempt one connection, resetting partial bytes but not integrated odometry.
  bool connect();
  /// Reset sampling origin after the initial port-open attempt.
  /// @throws std::invalid_argument if time is nonfinite.
  void resetSamplingTime(double now);
  /// Poll once (normally every 10 ms); retry a closed port every 100 failed ticks.
  /// @throws std::invalid_argument if time is nonfinite.
  std::optional<DriverSample> tick(double now);
  /// Send one command; reject invalid values and attempt one zero frame instead.
  bool command(double vx, double vy, double wz);
  /// Attempt zero then close once. Later ticks/commands cannot reconnect or write.
  bool shutdown();
  /// Return last accepted sample, absent until a complete valid frame was read.
  const std::optional<DriverSample> & latest() const;
  /// Return lifetime transmit/reconnect counters.
  const DriverCounters & counters() const;
  /// Return lifetime receive/framing counters.
  const ReceiveCounters & receiveCounters() const;
  /// Return the most recent I/O or command-rejection error.
  const std::string & lastError() const;
  /// Report the transport's open state without initiating a connection.
  bool online() const;

private:
  bool writeFrame(const CommandFrame & frame);
  ByteStream & stream_;
  std::array<float, 4> scales_;
  double last_time_;
  std::array<float, 3> pose_{};
  StatusReader reader_;
  DriverCounters counters_{};
  std::optional<DriverSample> latest_;
  std::string last_error_;
  uint64_t reconnect_divider_{};
  unsigned int voltage_divider_{};
  bool stopped_{};
};

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__DRIVER_CORE_HPP_
