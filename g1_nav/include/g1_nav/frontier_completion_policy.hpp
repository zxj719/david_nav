#pragma once

#include <cmath>
#include <optional>
#include <utility>

namespace g1_nav
{

inline bool is_same_frontier_anchor(
  const std::pair<double, double> & lhs,
  const std::pair<double, double> & rhs,
  double match_radius)
{
  const double radius = std::max(0.0, match_radius);
  return std::hypot(lhs.first - rhs.first, lhs.second - rhs.second) <= radius;
}

inline bool should_complete_candidate_frontier_by_proximity(
  const std::optional<std::pair<double, double>> & current_frontier,
  const std::pair<double, double> & candidate_anchor,
  const std::pair<double, double> & robot_xy,
  double reached_radius,
  double anchor_match_radius)
{
  if (!current_frontier) {
    return false;
  }

  return is_same_frontier_anchor(
           *current_frontier, candidate_anchor, anchor_match_radius) &&
         std::hypot(
           candidate_anchor.first - robot_xy.first,
           candidate_anchor.second - robot_xy.second) <= std::max(0.0, reached_radius);
}

}  // namespace g1_nav
