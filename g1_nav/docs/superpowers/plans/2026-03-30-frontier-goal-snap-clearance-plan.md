# Frontier Goal Snap And Clearance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `frontier_explorer` choose a new frontier goal only when the candidate satisfies both `grid_map == 0` and `global_costmap/costmap == 0`, enforce an additional `0.35 m` clearance disk, snap only on a fixed ring with shared NMS radius, and switch Nav2 to a circular `0.35 m` robot footprint.

**Architecture:** Extract the snap-ring, dual-map admissibility, clearance, and non-maximum-suppression logic into a new pure C++ helper so we can lock behavior down with gtests before rewiring the node. Then integrate that helper into `frontier_explorer`, keep frontier anchors separate from navigation goals in runtime state, stop clearing blacklists immediately, and update YAML plus pytest coverage for the new parameters and circular footprint.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, `ament_cmake_pytest`, C++17, Python `pytest`, Nav2 costmap YAML

---

## File Structure

- `include/g1_nav/exploration_types.hpp`
  Shared map-view helpers. Add world-to-cell projection so the new selector can evaluate the same world point against both grids.
- `include/g1_nav/frontier_goal_selector.hpp`
  New pure interface for snap-ring target selection and frontier NMS.
- `src/frontier_goal_selector.cpp`
  New implementation for dual-map admissibility, clearance-disk checking, discrete ring candidate generation, robot-nearest candidate choice, and suppression.
- `src/frontier_explorer.cpp`
  Runtime integration: global costmap subscription, wait gating, anchor/goal separation, pending-goal handoff updates, blacklist behavior, and marker publishing.
- `config/frontier_explorer_params.yaml`
  Replace the removed direct-goal radius with `global_costmap_topic`, `frontier_snap_radius`, and `goal_clearance_radius`.
- `config/nav2_params_2d.yaml`
  Replace polygon footprints in both costmaps with `robot_radius: 0.35` and `footprint_padding: 0.0`.
- `test/frontier_goal_selector_test.cpp`
  New gtests for anchor admissibility, ring snapping, clearance-disk rejection, and NMS.
- `test/test_realsense_nav2_integration.py`
  Extend config assertions for the new frontier-explorer parameters and circular Nav2 footprint.
- `CMakeLists.txt`
  Build the new selector helper into both the executable and the gtest target.

### Task 1: Add Failing Goal-Selector Tests And Build Plumbing

**Files:**
- Modify: `g1_nav/CMakeLists.txt`
- Create: `g1_nav/include/g1_nav/frontier_goal_selector.hpp`
- Create: `g1_nav/src/frontier_goal_selector.cpp`
- Create: `g1_nav/test/frontier_goal_selector_test.cpp`

- [ ] **Step 1: Write the failing selector tests**

```cpp
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
  double resolution = 0.5)
{
  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = width;
  grid.height = height;
  grid.resolution = resolution;
  return grid;
}

g1_nav::Frontier make_frontier(double x, double y, double cost, double heuristic_distance, int size = 10)
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
  config.goal_clearance_radius = 0.35;
  return config;
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
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 1.5);
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
```

- [ ] **Step 2: Run the new gtest target to verify it fails**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_goal_selector_test`

Expected: FAIL during build because `g1_nav/frontier_goal_selector.hpp` and the `frontier_goal_selector_test` target do not exist yet.

- [ ] **Step 3: Add minimal selector scaffolding and test plumbing**

```cpp
// include/g1_nav/frontier_goal_selector.hpp
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

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const Frontier & frontier,
  const std::pair<double, double> & robot_xy,
  const FrontierGoalSelectorConfig & config);

}  // namespace g1_nav
```

```cpp
// src/frontier_goal_selector.cpp
#include "g1_nav/frontier_goal_selector.hpp"

namespace g1_nav
{

std::vector<Frontier> suppress_frontiers_by_radius(
  const std::vector<Frontier> & frontiers,
  double)
{
  return frontiers;
}

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView &,
  const GridMapView &,
  const Frontier &,
  const std::pair<double, double> &,
  const FrontierGoalSelectorConfig &)
{
  return std::nullopt;
}

}  // namespace g1_nav
```

```cmake
add_library(frontier_goal_selector_lib
  src/frontier_goal_selector.cpp
)
target_compile_features(frontier_goal_selector_lib PUBLIC cxx_std_17)
target_include_directories(frontier_goal_selector_lib PUBLIC include)

target_link_libraries(frontier_explorer_cpp frontier_goal_selector_lib)

if(BUILD_TESTING)
  ament_add_gtest(
    frontier_goal_selector_test
    test/frontier_goal_selector_test.cpp
  )
  target_compile_features(frontier_goal_selector_test PUBLIC cxx_std_17)
  target_include_directories(frontier_goal_selector_test PUBLIC include)
  target_link_libraries(frontier_goal_selector_test frontier_goal_selector_lib)
endif()
```

- [ ] **Step 4: Run the selector tests again to verify they now fail for behavior**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_goal_selector_test`

Expected: FAIL in `UsesAnchorWhenAnchorIsAlreadyAdmissible`, `SnapsToRobotNearestAdmissibleRingCandidate`, and `SuppressesLowerRankedFrontiersInsideSnapRadius` because the scaffold returns `std::nullopt` and does not suppress anything.

- [ ] **Step 5: Commit the red test baseline**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add CMakeLists.txt include/g1_nav/frontier_goal_selector.hpp src/frontier_goal_selector.cpp test/frontier_goal_selector_test.cpp
git commit -m "test: add frontier goal selector coverage"
```

### Task 2: Implement Dual-Map Snap Selection, Clearance, And NMS

**Files:**
- Modify: `g1_nav/include/g1_nav/exploration_types.hpp`
- Modify: `g1_nav/include/g1_nav/frontier_goal_selector.hpp`
- Modify: `g1_nav/src/frontier_goal_selector.cpp`
- Test: `g1_nav/test/frontier_goal_selector_test.cpp`

- [ ] **Step 1: Add the failing clearance-disk regression test**

```cpp
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
```

- [ ] **Step 2: Run the selector tests to verify the new regression is red**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_goal_selector_test`

Expected: FAIL because `select_frontier_target()` still has no real admissibility, ring-search, or clearance implementation.

- [ ] **Step 3: Implement map projection helpers and the selector logic**

```cpp
// include/g1_nav/exploration_types.hpp
#include <cmath>
#include <optional>

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
```

```cpp
// src/frontier_goal_selector.cpp
#include "g1_nav/frontier_goal_selector.hpp"

#include <algorithm>
#include <cmath>

namespace g1_nav
{
namespace
{

bool is_goal_admissible(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const std::pair<double, double> & world_xy,
  double goal_clearance_radius)
{
  const auto grid_cell = grid_map.world_to_cell(world_xy.first, world_xy.second);
  const auto global_cell = global_costmap.world_to_cell(world_xy.first, world_xy.second);
  if (!grid_cell || !global_cell) {
    return false;
  }

  if (!grid_map.is_goal_free(grid_cell->first, grid_cell->second) ||
    !global_costmap.is_goal_free(global_cell->first, global_cell->second))
  {
    return false;
  }

  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(goal_clearance_radius / global_costmap.resolution)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int x = global_cell->first + dx;
      const int y = global_cell->second + dy;
      if (!global_costmap.in_bounds(x, y)) {
        return false;
      }

      const auto candidate = global_costmap.cell_center_world(x, y);
      if (std::hypot(candidate.first - world_xy.first, candidate.second - world_xy.second) <=
        goal_clearance_radius &&
        !global_costmap.is_goal_free(x, y))
      {
        return false;
      }
    }
  }

  return true;
}

std::vector<std::pair<double, double>> ring_candidates(
  const GridMapView & global_costmap,
  const std::pair<double, double> & anchor_xy,
  double snap_radius)
{
  std::vector<std::pair<double, double>> candidates;
  const auto anchor_cell = global_costmap.world_to_cell(anchor_xy.first, anchor_xy.second);
  if (!anchor_cell) {
    return candidates;
  }

  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(snap_radius / global_costmap.resolution)));
  const double tolerance = 0.5 * global_costmap.resolution + 1e-9;

  for (int dy = -radius_cells - 1; dy <= radius_cells + 1; ++dy) {
    for (int dx = -radius_cells - 1; dx <= radius_cells + 1; ++dx) {
      const int x = anchor_cell->first + dx;
      const int y = anchor_cell->second + dy;
      if (!global_costmap.in_bounds(x, y)) {
        continue;
      }

      const auto world_xy = global_costmap.cell_center_world(x, y);
      const double distance = std::hypot(
        world_xy.first - anchor_xy.first,
        world_xy.second - anchor_xy.second);
      if (std::abs(distance - snap_radius) <= tolerance) {
        candidates.push_back(world_xy);
      }
    }
  }

  return candidates;
}

}  // namespace

std::vector<Frontier> suppress_frontiers_by_radius(
  const std::vector<Frontier> & frontiers,
  double suppression_radius)
{
  std::vector<Frontier> kept;
  kept.reserve(frontiers.size());

  for (const auto & frontier : frontiers) {
    bool suppressed = false;
    for (const auto & kept_frontier : kept) {
      if (std::hypot(
          frontier.centroid_x - kept_frontier.centroid_x,
          frontier.centroid_y - kept_frontier.centroid_y) <= suppression_radius)
      {
        suppressed = true;
        break;
      }
    }

    if (!suppressed) {
      kept.push_back(frontier);
    }
  }

  return kept;
}

std::optional<FrontierTarget> select_frontier_target(
  const GridMapView & grid_map,
  const GridMapView & global_costmap,
  const Frontier & frontier,
  const std::pair<double, double> & robot_xy,
  const FrontierGoalSelectorConfig & config)
{
  const std::pair<double, double> anchor_xy{frontier.centroid_x, frontier.centroid_y};
  if (is_goal_admissible(grid_map, global_costmap, anchor_xy, config.goal_clearance_radius)) {
    return FrontierTarget{frontier, anchor_xy, anchor_xy};
  }

  auto candidates = ring_candidates(global_costmap, anchor_xy, config.snap_radius);
  std::sort(
    candidates.begin(), candidates.end(),
    [&robot_xy](const auto & a, const auto & b) {
      return std::hypot(a.first - robot_xy.first, a.second - robot_xy.second) <
             std::hypot(b.first - robot_xy.first, b.second - robot_xy.second);
    });

  for (const auto & candidate : candidates) {
    if (is_goal_admissible(grid_map, global_costmap, candidate, config.goal_clearance_radius)) {
      return FrontierTarget{frontier, anchor_xy, candidate};
    }
  }

  return std::nullopt;
}

}  // namespace g1_nav
```

- [ ] **Step 4: Run the focused selector gtests to verify they pass**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_goal_selector_test`

Expected: PASS for all tests in `frontier_goal_selector_test`.

- [ ] **Step 5: Run the existing frontier-search gtests to catch collateral regressions**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test`

Expected: PASS because frontier-search semantics were not changed by the selector helper.

- [ ] **Step 6: Commit the pure helper implementation**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add include/g1_nav/exploration_types.hpp include/g1_nav/frontier_goal_selector.hpp src/frontier_goal_selector.cpp test/frontier_goal_selector_test.cpp
git commit -m "feat: add frontier goal selector"
```

### Task 3: Add Config Tests And Update YAML For Snap Radius And Circular Footprint

**Files:**
- Modify: `g1_nav/config/frontier_explorer_params.yaml`
- Modify: `g1_nav/config/nav2_params_2d.yaml`
- Modify: `g1_nav/test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Extend the pytest coverage with the new configuration expectations**

```python
def test_frontier_explorer_uses_global_costmap_snap_parameters():
    with (REPO_ROOT / 'config/frontier_explorer_params.yaml').open() as stream:
        config = yaml.safe_load(stream)

    params = config['frontier_explorer']['ros__parameters']

    assert params['global_costmap_topic'] == '/global_costmap/costmap'
    assert params['frontier_snap_radius'] == 1.0
    assert params['goal_clearance_radius'] == 0.35
    assert 'direct_frontier_goal_search_radius' not in params


def test_nav2_costmaps_use_circular_robot_radius():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    for costmap_name in ('local_costmap', 'global_costmap'):
        params = config[costmap_name][costmap_name]['ros__parameters']

        assert params['robot_radius'] == 0.35
        assert params['footprint_padding'] == 0.0
        assert 'footprint' not in params
```

- [ ] **Step 2: Run the pytest target to verify it fails on the current YAML**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R realsense_nav2_integration_test`

Expected: FAIL because `frontier_explorer_params.yaml` still contains `direct_frontier_goal_search_radius`, and both Nav2 costmaps still use polygon `footprint` settings.

- [ ] **Step 3: Update the frontier and Nav2 YAML files**

```yaml
# config/frontier_explorer_params.yaml
frontier_explorer:
  ros__parameters:
    map_topic: /lightning/grid_map
    global_costmap_topic: /global_costmap/costmap
    live_map_ready_topic: /map_live_ready
    require_live_map: true
    frontier_update_radius: 0.0
    frontier_snap_radius: 1.0
    goal_clearance_radius: 0.35
    goal_update_min_distance: 0.4
```

```yaml
# config/nav2_params_2d.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.35
      footprint_padding: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.35
      footprint_padding: 0.0
```

Delete the old `footprint:` entries from both costmap sections and remove `direct_frontier_goal_search_radius` from `frontier_explorer_params.yaml`.

- [ ] **Step 4: Run the pytest target again to verify the YAML is green**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R realsense_nav2_integration_test`

Expected: PASS for the integration-style config tests.

- [ ] **Step 5: Commit the config changes**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add config/frontier_explorer_params.yaml config/nav2_params_2d.yaml test/test_realsense_nav2_integration.py
git commit -m "chore: configure circular nav footprint"
```

### Task 4: Integrate The Selector Into `frontier_explorer`

**Files:**
- Modify: `g1_nav/src/frontier_explorer.cpp`
- Verify: `g1_nav/include/g1_nav/frontier_goal_selector.hpp`
- Verify: `g1_nav/src/frontier_goal_selector.cpp`

- [ ] **Step 1: Add the new parameters, costmap cache, and pending-goal structure**

```cpp
#include "g1_nav/frontier_goal_selector.hpp"

struct PendingNavigationGoal
{
  std::optional<std::pair<double, double>> frontier_xy;
  std::pair<double, double> goal_xy;
  std::optional<double> yaw;
  bool returning_to_initial_pose{false};
};

declare_parameter("global_costmap_topic", "/global_costmap/costmap");
declare_parameter("frontier_snap_radius", 1.0);
declare_parameter("goal_clearance_radius", 0.35);

global_costmap_topic_ = get_parameter("global_costmap_topic").as_string();
frontier_snap_radius_ = std::max(0.1, get_parameter("frontier_snap_radius").as_double());
goal_clearance_radius_ = std::max(0.0, get_parameter("goal_clearance_radius").as_double());

global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
  global_costmap_topic_, map_qos,
  std::bind(&FrontierExplorerNode::global_costmap_callback, this, std::placeholders::_1));

void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  global_costmap_msg_ = msg;
}
```

- [ ] **Step 2: Gate planning on the latest cached global costmap and select targets from ranked frontier anchors**

```cpp
if (!global_costmap_msg_) {
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Waiting for global costmap data on %s", global_costmap_topic_.c_str());
  return;
}

const auto grid = make_grid_view();
const auto global_costmap = make_global_costmap_view();
if (!grid.valid() || !global_costmap.valid()) {
  return;
}

auto active_frontiers = filter_frontiers(observed_frontiers);
if (!active_frontiers.empty()) {
  sort_frontiers_for_selection(active_frontiers, *robot_xy);
  active_frontiers = g1_nav::suppress_frontiers_by_radius(active_frontiers, frontier_snap_radius_);
}

std::vector<g1_nav::FrontierTarget> targets;
targets.reserve(active_frontiers.size());
g1_nav::FrontierGoalSelectorConfig selector_config;
selector_config.snap_radius = frontier_snap_radius_;
selector_config.goal_clearance_radius = goal_clearance_radius_;

for (const auto & frontier : active_frontiers) {
  const auto target = g1_nav::select_frontier_target(
    grid, global_costmap, frontier, *robot_xy, selector_config);
  if (!target) {
    blacklist_point(
      frontier.centroid_x,
      frontier.centroid_y,
      "No admissible navigation goal on the snap ring");
    continue;
  }
  targets.push_back(*target);
}

g1_nav::GridMapView make_global_costmap_view() const
{
  g1_nav::GridMapView grid;
  if (!global_costmap_msg_) {
    return grid;
  }
  grid.cells = &global_costmap_msg_->data;
  grid.width = static_cast<int>(global_costmap_msg_->info.width);
  grid.height = static_cast<int>(global_costmap_msg_->info.height);
  grid.resolution = global_costmap_msg_->info.resolution;
  grid.origin_x = global_costmap_msg_->info.origin.position.x;
  grid.origin_y = global_costmap_msg_->info.origin.position.y;
  return grid;
}
```

- [ ] **Step 3: Keep anchor and goal separate during preemption, completion, logging, and blacklist handling**

```cpp
void request_navigation_preempt(
  const std::optional<std::pair<double, double>> & frontier_xy,
  const std::pair<double, double> & goal_xy,
  std::optional<double> yaw,
  bool returning_to_initial_pose)
{
  pending_goal_ = PendingNavigationGoal{frontier_xy, goal_xy, yaw, returning_to_initial_pose};
  if (!navigating_) {
    dispatch_pending_goal();
    return;
  }
  preempt_requested_ = true;
  cancel_current_goal();
}

void dispatch_pending_goal()
{
  if (!pending_goal_) {
    return;
  }

  const auto pending_goal = *pending_goal_;
  pending_goal_.reset();
  current_frontier_ = pending_goal.frontier_xy;
  returning_to_initial_pose_ = pending_goal.returning_to_initial_pose;
  navigate_to(pending_goal.goal_xy.first, pending_goal.goal_xy.second, pending_goal.yaw);
}
```

```cpp
for (const auto & target : targets) {
  if (distance_xy(*robot_xy, target.anchor_xy) <= frontier_reached_radius()) {
    complete_frontier(target.anchor_xy.first, target.anchor_xy.second);
    continue;
  }

  if (navigating_ && !should_preempt_navigation(target.anchor_xy, target.goal_xy)) {
    return;
  }

  if (navigating_) {
    request_navigation_preempt(
      target.anchor_xy,
      target.goal_xy,
      compute_goal_yaw(*robot_xy, target.goal_xy),
      false);
    return;
  }

  current_frontier_ = target.anchor_xy;
  navigate_to(
    target.goal_xy.first,
    target.goal_xy.second,
    compute_goal_yaw(*robot_xy, target.goal_xy));
  return;
}
```

Also remove this immediate blacklist reset block so ring-failure blacklists actually persist until timeout:

```cpp
if (active_frontiers.empty() && !blacklisted_.empty()) {
  blacklisted_.clear();
  active_frontiers = filter_frontiers(snapped_frontiers);
}
```

- [ ] **Step 4: Update marker publication and startup logging to reflect snapped goals instead of overwritten frontier centroids**

```cpp
RCLCPP_INFO(
  get_logger(),
  "Frontier explorer ready: map_topic=%s global_costmap_topic=%s snap_radius=%.2fm clearance=%.2fm",
  map_topic_.c_str(), global_costmap_topic_.c_str(), frontier_snap_radius_, goal_clearance_radius_);
```

```cpp
void publish_frontier_markers(
  const std::vector<g1_nav::FrontierTarget> & targets,
  const g1_nav::FrontierTarget & chosen)
{
  for (const auto & target : targets) {
    marker.pose.position.x = target.goal_xy.first;
    marker.pose.position.y = target.goal_xy.second;
    const bool selected =
      std::hypot(
        target.goal_xy.first - chosen.goal_xy.first,
        target.goal_xy.second - chosen.goal_xy.second) <= 1e-4;
  }
}
```

- [ ] **Step 5: Build the package to verify the runtime wiring compiles cleanly**

Run: `cd /home/unitree/ros2_ws && colcon build --packages-select g1_nav --event-handlers console_direct+`

Expected: PASS with `frontier_explorer_cpp` linked against `frontier_goal_selector_lib`.

- [ ] **Step 6: Commit the runtime integration**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add src/frontier_explorer.cpp
git commit -m "feat: integrate frontier goal snapping"
```

### Task 5: Run Final Verification

**Files:**
- Verify: `g1_nav/src/frontier_explorer.cpp`
- Verify: `g1_nav/src/frontier_goal_selector.cpp`
- Verify: `g1_nav/config/frontier_explorer_params.yaml`
- Verify: `g1_nav/config/nav2_params_2d.yaml`
- Verify: `g1_nav/test/frontier_goal_selector_test.cpp`
- Verify: `g1_nav/test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Run the focused C++ gtests**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R "frontier_goal_selector_test|frontier_search_test"`

Expected: PASS for both `frontier_goal_selector_test` and `frontier_search_test`.

- [ ] **Step 2: Run the Python integration-style config tests**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R realsense_nav2_integration_test`

Expected: PASS for `realsense_nav2_integration_test`.

- [ ] **Step 3: Rebuild the package once more from the final source tree**

Run: `cd /home/unitree/ros2_ws && colcon build --packages-select g1_nav --event-handlers console_direct+`

Expected: PASS with no compile or link failures.

- [ ] **Step 4: Inspect the final test report**

Run: `cd /home/unitree/ros2_ws && colcon test-result --verbose`

Expected: All `g1_nav` tests report `passed`; no lingering `frontier_goal_selector_test`, `frontier_search_test`, or `realsense_nav2_integration_test` failures remain.

- [ ] **Step 5: Commit the verified final state**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add CMakeLists.txt include/g1_nav/exploration_types.hpp include/g1_nav/frontier_goal_selector.hpp src/frontier_goal_selector.cpp src/frontier_explorer.cpp config/frontier_explorer_params.yaml config/nav2_params_2d.yaml test/frontier_goal_selector_test.cpp test/test_realsense_nav2_integration.py
git commit -m "feat: tighten frontier goal selection"
```
