#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "g1_nav/exploration_types.hpp"

namespace g1_nav
{

struct FrontierGoalSelectorConfig
{
  double snap_radius{1.0};
  double fallback_snap_radius{1.0};
  double goal_clearance_radius{0.35};
};

struct FrontierTarget
{
  Frontier frontier;
  std::pair<double, double> anchor_xy;
  std::pair<double, double> goal_xy;
};

std::vector<Frontier> suppress_frontiers_by_radius(
  const std::vector<Frontier> & frontiers,
  double suppression_radius);

void sort_frontier_targets_for_selection(
  std::vector<FrontierTarget> & targets,
  const std::pair<double, double> & robot_xy,
  double distance_weight,
  double size_weight);

std::vector<FrontierTarget> suppress_frontier_targets_by_radius(
  const std::vector<FrontierTarget> & targets,
  double suppression_radius);

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const Frontier & frontier,
  const std::pair<double, double> & robot_xy,
  const FrontierGoalSelectorConfig & config);

}  // namespace g1_nav
