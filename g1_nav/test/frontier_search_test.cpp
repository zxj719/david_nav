#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "g1_nav/frontier_search.hpp"

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

g1_nav::FrontierSearchConfig make_config()
{
  g1_nav::FrontierSearchConfig config;
  config.min_frontier_size = 1;
  config.frontier_update_radius = 0.0;
  return config;
}

}  // namespace

TEST(FrontierSearch, FindsFrontierOnSearchFreeProbabilityCells)
{
  const std::vector<int8_t> cells{
    100, 100, 100, 100, 100,
    100,  25,  25,  -1, 100,
    100,  25,  25,  -1, 100,
    100, 100, 100, 100, 100,
  };

  const auto grid = make_grid(cells, 5, 4);
  const std::vector<int8_t> costmap_cells(cells.size(), 0);
  const auto costmap = make_grid(costmap_cells, 5, 4);
  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, costmap, {1.5, 1.5}, std::vector<double>(cells.size(), 0.0), make_config());

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().size, 2);
  ASSERT_EQ(frontiers.front().cells.size(), 2u);
  EXPECT_EQ(frontiers.front().cells[0], std::make_pair(2, 1));
  EXPECT_EQ(frontiers.front().cells[1], std::make_pair(2, 2));
}

TEST(FrontierSearch, StartsFromNearestSearchFreeCellWhenRobotCellIsOccupied)
{
  const std::vector<int8_t> cells{
    100, 100, 100, 100, 100,
    100, 100,  40,  -1, 100,
    100, 100, 100, 100, 100,
  };

  const auto grid = make_grid(cells, 5, 3);
  const std::vector<int8_t> costmap_cells(cells.size(), 0);
  const auto costmap = make_grid(costmap_cells, 5, 3);
  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, costmap, {1.5, 1.5}, std::vector<double>(cells.size(), 0.0), make_config());

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().size, 1);
}

TEST(FrontierSearch, DoesNotTreatOccupiedProbabilityCellsAsFree)
{
  const std::vector<int8_t> cells{
    100, 100, 100, 100, 100,
    100,  51,  51,  -1, 100,
    100,  51,  51,  -1, 100,
    100, 100, 100, 100, 100,
  };

  const auto grid = make_grid(cells, 5, 4);
  const std::vector<int8_t> costmap_cells(cells.size(), 0);
  const auto costmap = make_grid(costmap_cells, 5, 4);
  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, costmap, {1.5, 1.5}, std::vector<double>(cells.size(), 0.0), make_config());

  EXPECT_TRUE(frontiers.empty());
}

TEST(FrontierSearch, FiltersHighCostGlobalCostmapCellsDuringFrontierSearch)
{
  const std::vector<int8_t> grid_cells{
    100, 100, 100, 100, 100, 100, 100,
    100,   0,   0,   0,   0,   0, 100,
    100,  -1,  -1,  -1,  -1,  -1, 100,
    100, 100, 100, 100, 100, 100, 100,
  };
  const std::vector<int8_t> costmap_cells{
      0,   0,   0,   0,   0,   0,   0,
      0,   0,   0, 100,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,
  };

  auto config = make_config();
  config.costmap_search_threshold = 20;

  const auto grid = make_grid(grid_cells, 7, 4);
  const auto costmap = make_grid(costmap_cells, 7, 4);
  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, costmap, {1.5, 1.5}, std::vector<double>(grid_cells.size(), 0.0), config);

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().size, 2);
  EXPECT_EQ(frontiers.front().cells.front(), std::make_pair(1, 1));
  EXPECT_EQ(frontiers.front().cells.back(), std::make_pair(2, 1));
}

TEST(FrontierSearch, KeepsLowCostGlobalCostmapCellsSearchable)
{
  const std::vector<int8_t> grid_cells{
    100, 100, 100, 100, 100, 100, 100,
    100,   0,   0,   0,   0,   0, 100,
    100,  -1,  -1,  -1,  -1,  -1, 100,
    100, 100, 100, 100, 100, 100, 100,
  };
  const std::vector<int8_t> costmap_cells{
      0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,  10,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,
  };

  auto config = make_config();
  config.costmap_search_threshold = 20;

  const auto grid = make_grid(grid_cells, 7, 4);
  const auto costmap = make_grid(costmap_cells, 7, 4);
  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, costmap, {1.5, 1.5}, std::vector<double>(grid_cells.size(), 0.0), config);

  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers.front().size, 5);
}

TEST(GridMapViewSemantics, DistinguishesUnknownSearchFreeGoalFreeAndObstacle)
{
  const std::vector<int8_t> cells{-1, 0, 25, 50, 51, 100};
  const auto grid = make_grid(cells, 6, 1);

  EXPECT_TRUE(grid.is_unknown(0, 0));
  EXPECT_TRUE(grid.is_goal_free(1, 0));
  EXPECT_TRUE(grid.is_search_free(1, 0));
  EXPECT_TRUE(grid.is_search_free(2, 0));
  EXPECT_TRUE(grid.is_search_free(3, 0));
  EXPECT_FALSE(grid.is_goal_free(2, 0));
  EXPECT_FALSE(grid.is_search_free(4, 0));
  EXPECT_TRUE(grid.is_obstacle(4, 0));
  EXPECT_TRUE(grid.is_obstacle(5, 0));
}
