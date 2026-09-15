// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "wheeltec_driver/diagnostics.hpp"

#include <cmath>
#include <stdexcept>

namespace wheeltec_driver
{

diagnostic_msgs::msg::DiagnosticArray makeDiagnostics(
  const DriverCore & core, double receive_hz, const std::string & hardware_id,
  const builtin_interfaces::msg::Time & stamp)
{
  if (!std::isfinite(receive_hz) || receive_hz < 0.0) {
    throw std::invalid_argument("diagnostic receive rate must be finite and nonnegative");
  }
  diagnostic_msgs::msg::DiagnosticArray result;
  result.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "wheeltec_driver/serial";
  status.hardware_id = hardware_id;
  const bool online = core.online();
  status.level = online ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = online ? "serial online" : "serial offline";
  if (online && !core.latest()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "serial online; awaiting first valid status frame";
  }
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
  const auto & rx = core.receiveCounters();
  const auto & tx = core.counters();
  add("serial_online", online ? "true" : "false");
  add("receive_hz", std::to_string(receive_hz));
  add("valid_frames", std::to_string(rx.valid_frames));
  add("checksum_errors", std::to_string(rx.checksum_errors));
  add("framing_errors", std::to_string(rx.framing_errors));
  add("read_errors", std::to_string(rx.io_errors));
  add("write_errors", std::to_string(tx.transmit_errors));
  add(
    "reconnect_attempts",
    std::to_string(tx.connection_attempts ? tx.connection_attempts - 1 : 0));
  add("successful_connections", std::to_string(tx.successful_connections));
  add("watchdog_stops", std::to_string(tx.watchdog_stops));
  add("rejected_commands", std::to_string(tx.rejected_commands));
  add("voltage", core.latest() ? std::to_string(core.latest()->sensor.voltage) : "unavailable");
  add("Flag_Stop", core.latest() ? std::to_string(core.latest()->sensor.flag_stop) : "unavailable");
  result.status.push_back(status);
  return result;
}

}  // namespace wheeltec_driver
