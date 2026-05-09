#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "g1_nav/frontier_goal_selector.hpp"

namespace
{

g1_nav::GridMapView make_grid(
  const std::vector<int8_t> & cells,
  int width,
  int height,
  double resolution = 1.0)
{
  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  return grid;
}

g1_nav::Frontier make_frontier(
  double x,
  double y,
  double cost,
  double heuristic_distance,
  int size = 10)
{
  g1_nav::Frontier frontier;
  frontier.centroid_x = x;
  frontier.centroid_y = y;
  frontier.cost = cost;
  frontier.heuristic_distance = heuristic_distance;
  frontier.size = size;
  return frontier;
}

g1_nav::FrontierGoalSelectorConfig make_config()
{
  g1_nav::FrontierGoalSelectorConfig config;
  config.snap_radius = 1.0;
  config.fallback_snap_radius = 1.0;
  config.goal_clearance_radius = 0.35;
  return config;
}

g1_nav::FrontierTarget make_target(
  double anchor_x,
  double anchor_y,
  double goal_x,
  double goal_y,
  int size = 10,
  double heuristic_distance = 0.0)
{
  const auto frontier = make_frontier(
    anchor_x, anchor_y, 0.0, heuristic_distance, size);
  return g1_nav::FrontierTarget{
    frontier,
    {anchor_x, anchor_y},
    {goal_x, goal_y}};
}

}  // namespace

TEST(FrontierGoalSelector, UsesAnchorWhenAnchorIsAlreadyAdmissible)
{
  const std::vector<int8_t> grid_cells(25, 0);
  const std::vector<int8_t> global_costmap_cells(25, 0);
  const auto grid = make_grid(grid_cells, 5, 5);
  const auto global_costmap = make_grid(global_costmap_cells, 5, 5);
  const auto frontier = make_frontier(1.5, 1.5, 3.0, 2.0);

  const auto selected = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, {0.0, 0.0}, make_config());

  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected->anchor_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->anchor_xy.second, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 1.5);
}

TEST(FrontierGoalSelector, SnapsToRobotNearestAdmissibleRingCandidate)
{
  const std::vector<int8_t> grid_cells{
    100, 100,   0, 100, 100,
    100, 100, 100, 100, 100,
      0, 100,   0, 100,   0,
    100, 100, 100, 100, 100,
    100, 100,   0, 100, 100,
  };
  const std::vector<int8_t> global_costmap_cells{
    100, 100,   0, 100, 100,
    100, 100, 100, 100, 100,
      0, 100, 100, 100,   0,
    100, 100, 100, 100, 100,
    100, 100, 100, 100, 100,
  };
  const auto grid = make_grid(grid_cells, 5, 5);
  const auto global_costmap = make_grid(global_costmap_cells, 5, 5);
  const auto frontier = make_frontier(1.5, 1.5, 5.0, 4.0);

  auto config = make_config();
  config.goal_clearance_radius = 0.0;

  const auto selected = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, {0.0, 1.5}, config);

  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected->anchor_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->anchor_xy.second, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.first, 0.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 2.5);
}

TEST(FrontierGoalSelector, SuppressesLowerRankedFrontiersInsideSnapRadius)
{
  std::vector<g1_nav::Frontier> frontiers{
    make_frontier(1.0, 1.0, 9.0, 1.0),
    make_frontier(1.6, 1.0, 8.0, 1.1),
    make_frontier(3.5, 3.5, 7.0, 4.0),
  };

  const auto kept = g1_nav::suppress_frontiers_by_radius(frontiers, 1.0);

  ASSERT_EQ(kept.size(), 2u);
  EXPECT_DOUBLE_EQ(kept[0].centroid_x, 1.0);
  EXPECT_DOUBLE_EQ(kept[0].centroid_y, 1.0);
  EXPECT_DOUBLE_EQ(kept[1].centroid_x, 3.5);
  EXPECT_DOUBLE_EQ(kept[1].centroid_y, 3.5);
}

TEST(FrontierGoalSelector, SortsTargetsByNearestLiveGoalDistance)
{
  std::vector<g1_nav::FrontierTarget> targets{
    make_target(1.0, 0.0, 6.0, 0.0, 10, 1.0),
    make_target(4.0, 0.0, 2.0, 0.0, 10, 4.0),
    make_target(3.0, 0.0, 4.0, 0.0, 10, 3.0),
  };

  g1_nav::sort_frontier_targets_for_selection(targets, {0.0, 0.0}, 1.0, 0.0);

  ASSERT_EQ(targets.size(), 3u);
  EXPECT_DOUBLE_EQ(targets[0].anchor_xy.first, 4.0);
  EXPECT_DOUBLE_EQ(targets[0].goal_xy.first, 2.0);
  EXPECT_DOUBLE_EQ(targets[1].anchor_xy.first, 3.0);
  EXPECT_DOUBLE_EQ(targets[1].goal_xy.first, 4.0);
  EXPECT_DOUBLE_EQ(targets[2].anchor_xy.first, 1.0);
  EXPECT_DOUBLE_EQ(targets[2].goal_xy.first, 6.0);
}

TEST(FrontierGoalSelector, SuppressesLowerRankedTargetsInsideSnapRadiusAfterGoalSorting)
{
  std::vector<g1_nav::FrontierTarget> targets{
    make_target(1.0, 1.0, 5.0, 1.0, 10, 1.0),
    make_target(1.6, 1.0, 0.5, 1.0, 10, 1.6),
    make_target(4.0, 4.0, 2.0, 4.0, 10, 4.0),
  };

  g1_nav::sort_frontier_targets_for_selection(targets, {0.0, 0.0}, 1.0, 0.0);
  const auto kept = g1_nav::suppress_frontier_targets_by_radius(targets, 1.0);

  ASSERT_EQ(kept.size(), 2u);
  EXPECT_DOUBLE_EQ(kept[0].anchor_xy.first, 1.6);
  EXPECT_DOUBLE_EQ(kept[0].anchor_xy.second, 1.0);
  EXPECT_DOUBLE_EQ(kept[0].goal_xy.first, 0.5);
  EXPECT_DOUBLE_EQ(kept[0].goal_xy.second, 1.0);
  EXPECT_DOUBLE_EQ(kept[1].anchor_xy.first, 4.0);
  EXPECT_DOUBLE_EQ(kept[1].anchor_xy.second, 4.0);
}

TEST(FrontierGoalSelector, RejectsRingCandidateWhoseClearanceDiskTouchesCostmapInflation)
{
  const std::vector<int8_t> grid_cells{
    100, 100,   0, 100, 100,
    100, 100,   0, 100, 100,
      0, 100, 100, 100, 100,
    100, 100,   0, 100, 100,
    100, 100, 100, 100, 100,
  };
  const std::vector<int8_t> global_costmap_cells{
    100, 100, 100, 100, 100,
    100, 100,   5, 100, 100,
      0, 100, 100, 100, 100,
    100, 100, 100, 100, 100,
    100, 100, 100, 100, 100,
  };
  const auto grid = make_grid(grid_cells, 5, 5);
  const auto global_costmap = make_grid(global_costmap_cells, 5, 5);
  const auto frontier = make_frontier(1.5, 1.5, 5.0, 4.0);

  auto config = make_config();
  config.goal_clearance_radius = 0.6;

  const auto selected = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, {0.0, 1.5}, config);

  EXPECT_FALSE(selected.has_value());
}

TEST(FrontierGoalSelector, UsesFallbackSnapRadiusWhenPrimaryRingHasNoGoal)
{
  const std::vector<int8_t> grid_cells{
    100, 100, 100,   0, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100,
    100,   0, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100,
  };
  const auto grid = make_grid(grid_cells, 7, 7);
  const auto global_costmap = make_grid(grid_cells, 7, 7);
  const auto frontier = make_frontier(3.5, 3.5, 5.0, 2.0);

  auto config = make_config();
  config.goal_clearance_radius = 0.0;
  config.snap_radius = 1.0;
  config.fallback_snap_radius = 2.0;

  const auto selected = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, {0.0, 3.5}, config);

  ASSERT_TRUE(selected.has_value());
  EXPECT_DOUBLE_EQ(selected->anchor_xy.first, 3.5);
  EXPECT_DOUBLE_EQ(selected->anchor_xy.second, 3.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 3.5);
}
