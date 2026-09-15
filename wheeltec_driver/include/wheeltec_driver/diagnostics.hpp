// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WHEELTEC_DRIVER__DIAGNOSTICS_HPP_
#define WHEELTEC_DRIVER__DIAGNOSTICS_HPP_

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "wheeltec_driver/driver_core.hpp"

namespace wheeltec_driver
{

/// Create a serial-health snapshot without a ROS context or any transport writes.
/// No voltage threshold or Flag_Stop meaning is inferred from undocumented firmware.
diagnostic_msgs::msg::DiagnosticArray makeDiagnostics(
  const DriverCore & core, double receive_hz, const std::string & hardware_id,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__DIAGNOSTICS_HPP_
