#include "g1_nav/frontier_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace g1_nav
{
namespace
{

constexpr std::array<std::pair<int, int>, 8> kNeighbors8{{
  std::pair<int, int>{-1, -1},
  std::pair<int, int>{-1, 0},
  std::pair<int, int>{-1, 1},
  std::pair<int, int>{0, -1},
  std::pair<int, int>{0, 1},
  std::pair<int, int>{1, -1},
  std::pair<int, int>{1, 0},
  std::pair<int, int>{1, 1},
}};

bool is_costmap_search_free(
  const GridMapView & global_costmap,
  const std::pair<double, double> & world_xy,
  const FrontierSearchConfig & config)
{
  if (!global_costmap.valid()) {
    return true;
  }

  const auto costmap_cell = global_costmap.world_to_cell(world_xy.first, world_xy.second);
  return costmap_cell &&
         global_costmap.is_search_free(
           costmap_cell->first, costmap_cell->second, config.costmap_search_threshold);
}

bool is_search_cell(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  int x,
  int y,
  const FrontierSearchConfig & config)
{
  if (!grid.is_search_free(x, y, config.search_free_threshold)) {
    return false;
  }

  return is_costmap_search_free(global_costmap, grid.cell_center_world(x, y), config);
}

bool is_frontier_cell(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  int x,
  int y,
  const FrontierSearchConfig & config)
{
  // A frontier is a traversable search cell bordering unknown space. When a
  // global costmap is available, also reject cells that sit inside high-cost
  // inflated obstacle shards so those cells cannot bridge one frontier cluster
  // through obstacle noise.
  if (!is_search_cell(grid, global_costmap, x, y, config)) {
    return false;
  }
  for (const auto & [dx, dy] : kNeighbors8) {
    const int nx = x + dx;
    const int ny = y + dy;
    if (grid.in_bounds(nx, ny) && grid.is_unknown(nx, ny)) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<int, int>> nearest_free_cell(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  int sx,
  int sy,
  int max_radius,
  const FrontierSearchConfig & config)
{
  for (int r = 1; r < max_radius; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy : {-r, r}) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (grid.in_bounds(nx, ny) &&
          is_search_cell(grid, global_costmap, nx, ny, config))
        {
          return std::make_pair(nx, ny);
        }
      }
    }
    for (int dy = -r + 1; dy < r; ++dy) {
      for (int dx : {-r, r}) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (grid.in_bounds(nx, ny) &&
          is_search_cell(grid, global_costmap, nx, ny, config))
        {
          return std::make_pair(nx, ny);
        }
      }
    }
  }
  return std::nullopt;
}

std::optional<std::pair<int, int>> nearest_frontier_cell(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  int sx,
  int sy,
  int max_radius,
  const FrontierSearchConfig & config)
{
  if (grid.in_bounds(sx, sy) && is_frontier_cell(grid, global_costmap, sx, sy, config)) {
    return std::make_pair(sx, sy);
  }

  for (int r = 1; r <= max_radius; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy : {-r, r}) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (grid.in_bounds(nx, ny) &&
          is_frontier_cell(grid, global_costmap, nx, ny, config))
        {
          return std::make_pair(nx, ny);
        }
      }
    }
    for (int dy = -r + 1; dy < r; ++dy) {
      for (int dx : {-r, r}) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (grid.in_bounds(nx, ny) &&
          is_frontier_cell(grid, global_costmap, nx, ny, config))
        {
          return std::make_pair(nx, ny);
        }
      }
    }
  }

  return std::nullopt;
}

Frontier build_frontier(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  int sx,
  int sy,
  std::vector<uint8_t> & state,
  const std::pair<double, double> & robot_xy,
  const std::vector<double> & dist_map,
  const FrontierSearchConfig & config)
{
  Frontier frontier;
  double sum_x = 0.0;
  double sum_y = 0.0;
  std::queue<std::pair<int, int>> queue;
  queue.emplace(sx, sy);
  state[static_cast<size_t>(grid.index(sx, sy))] = 3;

  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    const int current_idx = grid.index(cx, cy);
    if (state[static_cast<size_t>(current_idx)] == 4) {
      continue;
    }
    state[static_cast<size_t>(current_idx)] = 4;

    const auto [wx, wy] = grid.cell_world(cx, cy);
    sum_x += wx;
    sum_y += wy;
    frontier.size += 1;
    frontier.cells.emplace_back(cx, cy);
    frontier.min_distance = std::min(
      frontier.min_distance,
      std::hypot(wx - robot_xy.first, wy - robot_xy.second));

    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }
      const int nidx = grid.index(nx, ny);
      if (state[static_cast<size_t>(nidx)] != 3 &&
        state[static_cast<size_t>(nidx)] != 4 &&
        is_frontier_cell(grid, global_costmap, nx, ny, config))
      {
        state[static_cast<size_t>(nidx)] = 3;
        queue.emplace(nx, ny);
      }
    }
  }

  if (frontier.size > 0) {
    frontier.centroid_x = sum_x / frontier.size;
    frontier.centroid_y = sum_y / frontier.size;
    frontier.heuristic_distance = std::hypot(
      frontier.centroid_x - robot_xy.first,
      frontier.centroid_y - robot_xy.second);
    // Keep cost aligned with the selected frontier ordering metric so the
    // caller can continue logging a single comparable scalar.
    frontier.cost = frontier.heuristic_distance;
  }

  return frontier;
}

bool is_within_update_radius(
  const Frontier & frontier, const FrontierSearchConfig & config)
{
  return config.frontier_update_radius <= 0.0 ||
         frontier.min_distance <= config.frontier_update_radius;
}

}  // namespace

std::vector<Frontier> FrontierSearch::search(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  const std::pair<double, double> & robot_xy,
  const std::vector<double> & dist_map,
  const FrontierSearchConfig & config)
{
  std::vector<Frontier> frontiers;
  if (!grid.valid() || dist_map.size() != grid.cells->size()) {
    return frontiers;
  }

  int mx = static_cast<int>((robot_xy.first - grid.origin_x) / grid.resolution);
  int my = static_cast<int>((robot_xy.second - grid.origin_y) / grid.resolution);
  if (!grid.in_bounds(mx, my)) {
    return frontiers;
  }

  if (!is_search_cell(grid, global_costmap, mx, my, config)) {
    const auto free_cell = nearest_free_cell(grid, global_costmap, mx, my, 50, config);
    if (!free_cell) {
      return frontiers;
    }
    mx = free_cell->first;
    my = free_cell->second;
  }

  enum : uint8_t { UNVISITED = 0, MAP_OPEN = 1, MAP_CLOSED = 2, FRONTIER_OPEN = 3, FRONTIER_CLOSED = 4 };
  std::vector<uint8_t> state(static_cast<size_t>(grid.width * grid.height), UNVISITED);
  std::queue<std::pair<int, int>> bfs_queue;
  bfs_queue.emplace(mx, my);
  state[static_cast<size_t>(grid.index(mx, my))] = MAP_OPEN;

  while (!bfs_queue.empty()) {
    const auto [cx, cy] = bfs_queue.front();
    bfs_queue.pop();
    const int current_idx = grid.index(cx, cy);
    if (state[static_cast<size_t>(current_idx)] == MAP_CLOSED ||
      state[static_cast<size_t>(current_idx)] == FRONTIER_CLOSED)
    {
      continue;
    }

    if (state[static_cast<size_t>(current_idx)] != FRONTIER_OPEN &&
      is_frontier_cell(grid, global_costmap, cx, cy, config))
    {
      auto frontier = build_frontier(
        grid, global_costmap, cx, cy, state, robot_xy, dist_map, config);
      if (frontier.size >= config.min_frontier_size &&
        is_within_update_radius(frontier, config))
      {
        frontiers.push_back(frontier);
      }
    }

    if (state[static_cast<size_t>(current_idx)] != FRONTIER_CLOSED) {
      state[static_cast<size_t>(current_idx)] = MAP_CLOSED;
    }

    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }

      const int nidx = grid.index(nx, ny);
      if (state[static_cast<size_t>(nidx)] != FRONTIER_OPEN &&
        state[static_cast<size_t>(nidx)] != FRONTIER_CLOSED &&
        is_frontier_cell(grid, global_costmap, nx, ny, config))
      {
        auto frontier = build_frontier(
          grid, global_costmap, nx, ny, state, robot_xy, dist_map, config);
        if (frontier.size >= config.min_frontier_size &&
          is_within_update_radius(frontier, config))
        {
          frontiers.push_back(frontier);
        }
      }

      if (is_search_cell(grid, global_costmap, nx, ny, config) &&
        state[static_cast<size_t>(nidx)] == UNVISITED)
      {
        state[static_cast<size_t>(nidx)] = MAP_OPEN;
        bfs_queue.emplace(nx, ny);
      }
    }
  }

  const double weight_dist = config.frontier_distance_weight;
  const double weight_size = config.frontier_size_weight;
  std::sort(
    frontiers.begin(), frontiers.end(),
    [weight_dist, weight_size](const Frontier & a, const Frontier & b) {
      // Higher score is better: negative distance (closer = higher) + size contribution
      const double score_a = -a.heuristic_distance * weight_dist + static_cast<double>(a.size) * weight_size;
      const double score_b = -b.heuristic_distance * weight_dist + static_cast<double>(b.size) * weight_size;
      return score_a > score_b;
    });
  return frontiers;
}

std::optional<Frontier> FrontierSearch::revalidate_nearby_frontier(
  const GridMapView & grid,
  const GridMapView & global_costmap,
  const Frontier & frontier,
  const std::pair<double, double> & robot_xy,
  const std::vector<double> & dist_map,
  const FrontierSearchConfig & config,
  double match_radius)
{
  if (!grid.valid() || dist_map.size() != grid.cells->size()) {
    return std::nullopt;
  }

  const int mx = static_cast<int>(
    std::floor((frontier.centroid_x - grid.origin_x) / grid.resolution));
  const int my = static_cast<int>(
    std::floor((frontier.centroid_y - grid.origin_y) / grid.resolution));
  if (!grid.in_bounds(mx, my)) {
    return std::nullopt;
  }

  const int search_radius_cells = std::max(
    1, static_cast<int>(std::ceil(match_radius / grid.resolution)));
  const auto frontier_seed = nearest_frontier_cell(
    grid, global_costmap, mx, my, search_radius_cells, config);
  if (!frontier_seed) {
    return std::nullopt;
  }

  enum : uint8_t {
    kUnvisited = 0,
    kMapOpen = 1,
    kMapClosed = 2,
    kFrontierOpen = 3,
    kFrontierClosed = 4
  };
  std::vector<uint8_t> state(
    static_cast<size_t>(grid.width * grid.height), kUnvisited);
  Frontier validated = build_frontier(
    grid, global_costmap, frontier_seed->first, frontier_seed->second, state, robot_xy, dist_map,
    config);
  if (validated.size < config.min_frontier_size) {
    return std::nullopt;
  }

  const double centroid_shift = std::hypot(
    validated.centroid_x - frontier.centroid_x,
    validated.centroid_y - frontier.centroid_y);
  if (centroid_shift > match_radius) {
    return std::nullopt;
  }

  return validated;
}

}  // namespace g1_nav
