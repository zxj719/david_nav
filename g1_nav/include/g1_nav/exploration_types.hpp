#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace g1_nav
{

struct Frontier
{
  int size{0};
  double min_distance{std::numeric_limits<double>::infinity()};
  double heuristic_distance{std::numeric_limits<double>::infinity()};
  double cost{0.0};
  double centroid_x{0.0};
  double centroid_y{0.0};
  std::vector<std::pair<int, int>> cells;
};

struct GridMapView
{
  const std::vector<int8_t> * cells{nullptr};
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};

  bool valid() const
  {
    return cells != nullptr && !cells->empty() && width > 0 && height > 0;
  }

  bool in_bounds(int x, int y) const
  {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  int index(int x, int y) const
  {
    return y * width + x;
  }

  int8_t value(int x, int y) const
  {
    return (*cells)[static_cast<size_t>(index(x, y))];
  }

  bool is_unknown_value(int8_t cell_value) const
  {
    return cell_value < 0;
  }

  bool is_search_free_value(int8_t cell_value) const
  {
    return cell_value >= 0 && cell_value <= 50;
  }

  bool is_search_free_value(int8_t cell_value, int threshold) const
  {
    return cell_value >= 0 && cell_value <= threshold;
  }

  bool is_goal_free_value(int8_t cell_value) const
  {
    return cell_value == 0;
  }

  bool is_obstacle_value(int8_t cell_value) const
  {
    return cell_value > 50;
  }

  bool is_obstacle_value(int8_t cell_value, int threshold) const
  {
    return cell_value > threshold;
  }

  bool is_unknown(int x, int y) const
  {
    return is_unknown_value(value(x, y));
  }

  bool is_search_free(int x, int y) const
  {
    return is_search_free_value(value(x, y));
  }

  bool is_search_free(int x, int y, int threshold) const
  {
    return is_search_free_value(value(x, y), threshold);
  }

  bool is_goal_free(int x, int y) const
  {
    return is_goal_free_value(value(x, y));
  }

  bool is_obstacle(int idx) const
  {
    return is_obstacle_value((*cells)[static_cast<size_t>(idx)]);
  }

  bool is_obstacle(int x, int y) const
  {
    return is_obstacle_value(value(x, y));
  }

  bool is_obstacle(int x, int y, int threshold) const
  {
    return is_obstacle_value(value(x, y), threshold);
  }

  std::optional<std::pair<int, int>> world_to_cell(double wx, double wy) const
  {
    if (!valid()) {
      return std::nullopt;
    }

    const int x = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int y = static_cast<int>(std::floor((wy - origin_y) / resolution));
    if (!in_bounds(x, y)) {
      return std::nullopt;
    }
    return std::make_pair(x, y);
  }

  std::pair<double, double> cell_world(int x, int y) const
  {
    return {
      static_cast<double>(x) * resolution + origin_x,
      static_cast<double>(y) * resolution + origin_y};
  }

  std::pair<double, double> cell_center_world(int x, int y) const
  {
    return {
      (static_cast<double>(x) + 0.5) * resolution + origin_x,
      (static_cast<double>(y) + 0.5) * resolution + origin_y};
  }
};

}  // namespace g1_nav
