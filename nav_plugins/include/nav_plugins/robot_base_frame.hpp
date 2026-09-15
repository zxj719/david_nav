// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NAV_PLUGINS__ROBOT_BASE_FRAME_HPP_
#define NAV_PLUGINS__ROBOT_BASE_FRAME_HPP_

#include <optional>
#include <stdexcept>
#include <string>

namespace nav_plugins
{

/// Resolve a layer frame override, inheriting the parent costmap by default.
/// An explicitly empty frame is invalid rather than a silent fallback.
inline std::string resolveRobotBaseFrame(
  const std::string & costmap_frame,
  const std::optional<std::string> & layer_override = std::nullopt)
{
  const auto & effective = layer_override ? *layer_override : costmap_frame;
  if (effective.empty()) {
    throw std::invalid_argument("ScanPriorityLayer robot_base_frame must not be empty");
  }
  return effective;
}

}  // namespace nav_plugins

#endif  // NAV_PLUGINS__ROBOT_BASE_FRAME_HPP_
