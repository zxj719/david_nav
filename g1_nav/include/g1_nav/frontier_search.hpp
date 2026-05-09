#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "g1_nav/exploration_types.hpp"

namespace g1_nav
{

struct FrontierSearchConfig
{
  int min_frontier_size{5};
  int search_free_threshold{50};
  int costmap_search_threshold{100};
  double clearance_scale{0.3};
  double frontier_update_radius{3.0};
  double frontier_distance_weight{1.0};
  double frontier_size_weight{0.0};
};

class FrontierSearch
{
public:
  static std::vector<Frontier> search(
    const GridMapView & grid,
    const GridMapView & global_costmap,
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map,
    const FrontierSearchConfig & config);

  static std::optional<Frontier> revalidate_nearby_frontier(
    const GridMapView & grid,
    const GridMapView & global_costmap,
    const Frontier & frontier,
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map,
    const FrontierSearchConfig & config,
    double match_radius);
};

}  // namespace g1_nav
