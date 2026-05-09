# Probability-Map Frontier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Update frontier discovery to treat occupancy values `0..50` as search-free while keeping final navigation goals restricted to pure `0` cells.

**Architecture:** Introduce shared occupancy helpers on `GridMapView`, keep frontier search logic in place, and add focused gtests that lock in the split between search-free and goal-free semantics. `FrontierExplorerNode` will keep its runtime flow; its goal snapping logic will stay behaviorally unchanged and continue to require pure `0` cells.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, C++17

---

### Task 1: Add Failing Frontier-Semantics Tests And Test Plumbing

**Files:**
- Modify: `src/g1_nav/CMakeLists.txt`
- Modify: `src/g1_nav/package.xml`
- Create: `src/g1_nav/test/frontier_search_test.cpp`

- [ ] **Step 1: Write the failing tests**

```cpp
TEST(FrontierSearch, FindsFrontierOnSearchFreeProbabilityCells)
{
  const std::vector<int8_t> cells{
    100, 100, 100, 100, 100,
    100,  25,  25,  -1, 100,
    100,  25,  25,  -1, 100,
    100, 100, 100, 100, 100,
  };

  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = 5;
  grid.height = 4;
  grid.resolution = 1.0;

  g1_nav::FrontierSearchConfig config;
  config.min_frontier_size = 1;

  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, {1.5, 1.5}, std::vector<double>(cells.size(), 0.0), config);

  ASSERT_FALSE(frontiers.empty());
}

TEST(FrontierSearch, StartsFromNearestSearchFreeCellWhenRobotCellIsOccupied)
{
  const std::vector<int8_t> cells{
    100, 100, 100, 100, 100,
    100, 100,  40,  -1, 100,
    100, 100, 100, 100, 100,
  };

  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = 5;
  grid.height = 3;
  grid.resolution = 1.0;

  g1_nav::FrontierSearchConfig config;
  config.min_frontier_size = 1;

  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, {1.5, 1.5}, std::vector<double>(cells.size(), 0.0), config);

  ASSERT_EQ(frontiers.size(), 1u);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test`

Expected: FAIL because the gtest target does not exist yet, then because `FrontierSearch` still uses `value == 0`.

- [ ] **Step 3: Add minimal test/build plumbing**

```cmake
find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)
  ament_add_gtest(frontier_search_test
    test/frontier_search_test.cpp
    src/frontier_search.cpp
  )
  target_include_directories(frontier_search_test PUBLIC include)
endif()
```

- [ ] **Step 4: Run test to verify it still fails for the right reason**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test`

Expected: FAIL on frontier-search behavior, not on missing CMake targets.

- [ ] **Step 5: Commit**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add CMakeLists.txt package.xml test/frontier_search_test.cpp
git commit -m "test: add exploration semantics coverage"
```

### Task 2: Implement Shared Occupancy Semantics In GridMapView And FrontierSearch

**Files:**
- Modify: `src/g1_nav/include/g1_nav/exploration_types.hpp`
- Modify: `src/g1_nav/src/frontier_search.cpp`
- Modify: `src/g1_nav/src/frontier_explorer.cpp`
- Test: `src/g1_nav/test/frontier_search_test.cpp`

- [ ] **Step 1: Add the next failing assertions for obstacle rejection and helper semantics**

```cpp
TEST(FrontierSearch, DoesNotTreatOccupiedProbabilityCellsAsFree)
{
  const std::vector<int8_t> cells{
    100, 100, 100, 100, 100,
    100,  51,  51,  -1, 100,
    100,  51,  51,  -1, 100,
    100, 100, 100, 100, 100,
  };

  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = 5;
  grid.height = 4;
  grid.resolution = 1.0;

  g1_nav::FrontierSearchConfig config;
  config.min_frontier_size = 1;

  const auto frontiers = g1_nav::FrontierSearch::search(
    grid, {1.5, 1.5}, std::vector<double>(cells.size(), 0.0), config);

  EXPECT_TRUE(frontiers.empty());
}

TEST(GridMapViewSemantics, DistinguishesUnknownSearchFreeGoalFreeAndObstacle)
{
  const std::vector<int8_t> cells{
    -1, 0, 25, 50, 51, 100
  };

  g1_nav::GridMapView grid;
  grid.cells = &cells;
  grid.width = 6;
  grid.height = 1;

  EXPECT_TRUE(grid.is_unknown(0, 0));
  EXPECT_TRUE(grid.is_goal_free(1, 0));
  EXPECT_TRUE(grid.is_search_free(2, 0));
  EXPECT_TRUE(grid.is_search_free(3, 0));
  EXPECT_TRUE(grid.is_obstacle(4, 0));
  EXPECT_TRUE(grid.is_obstacle(5, 0));
  EXPECT_FALSE(grid.is_goal_free(2, 0));
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test`

Expected: FAIL because the helper methods do not exist yet and `FrontierSearch` still uses `value == 0`.

- [ ] **Step 3: Write minimal implementation**

```cpp
bool is_unknown_value(int8_t value) const { return value < 0; }
bool is_search_free_value(int8_t value) const { return value >= 0 && value <= 50; }
bool is_goal_free_value(int8_t value) const { return value == 0; }
bool is_obstacle_value(int8_t value) const { return value > 50; }

if (!grid.is_search_free(x, y)) {
  return false;
}

if (grid.in_bounds(nx, ny) && grid.is_unknown(nx, ny)) {
  return true;
}

if (!grid.is_search_free(mx, my)) {
  const auto free_cell = nearest_free_cell(...);
}

if (!grid.in_bounds(x, y) || !grid.is_goal_free(x, y)) {
  continue;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test`

Expected: PASS for the frontier-search tests.

- [ ] **Step 5: Commit**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add include/g1_nav/exploration_types.hpp src/frontier_search.cpp src/frontier_explorer.cpp test/frontier_search_test.cpp
git commit -m "feat: support probability map frontier search"
```

### Task 3: Run Final Verification

**Files:**
- Verify: `src/g1_nav/test/frontier_search_test.cpp`
- Verify: `src/g1_nav/src/frontier_search.cpp`
- Verify: `src/g1_nav/src/frontier_explorer.cpp`

- [ ] **Step 1: Run focused tests**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test`

Expected: PASS

- [ ] **Step 2: Run package build**

Run: `cd /home/unitree/ros2_ws && colcon build --packages-select g1_nav --event-handlers console_direct+`

Expected: SUCCEEDED for `g1_nav`

- [ ] **Step 3: Commit**

```bash
cd /home/unitree/ros2_ws/src/g1_nav
git add CMakeLists.txt package.xml include/g1_nav/exploration_types.hpp src/frontier_search.cpp src/frontier_explorer.cpp test/frontier_search_test.cpp
git commit -m "feat: update frontier search for probability maps"
```
