# Frontier Goal Lock And Fallback Snap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `frontier_explorer` keep the active navigation goal locked until it completes, add a larger fallback snap radius before blacklisting a frontier, and retry the same frontier once after navigation failure before blacklisting it.

**Architecture:** Extend the pure frontier-goal selector with a second snap ring so anchor selection stays deterministic and testable. Move the "keep current goal", "retry same frontier once", and "blacklist only after retry fails" decisions behind small pure policy helpers, then wire those helpers into `frontier_explorer` without disturbing unrelated search or visualization code.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, `ament_cmake_pytest`, C++17, Python `pytest`

---

## File Structure

- `include/g1_nav/frontier_goal_selector.hpp`
  Add `fallback_snap_radius` to selector config.
- `src/frontier_goal_selector.cpp`
  Try `anchor`, then primary snap ring, then fallback snap ring.
- `include/g1_nav/frontier_navigation_result_policy.hpp`
  Add pure helpers for "do not preempt active frontier navigation" and "retry same frontier once".
- `src/frontier_explorer.cpp`
  Stop reselecting/frontier-blacklisting while a goal is active, retry the same frontier once after abort or timeout cancellation, and blacklist only after retry failure.
- `config/frontier_explorer_params.yaml`
  Add `frontier_fallback_snap_radius`.
- `test/frontier_goal_selector_test.cpp`
  Cover fallback ring selection.
- `test/frontier_navigation_result_policy_test.cpp`
  Cover locked-goal and same-frontier retry decisions.
- `test/test_realsense_nav2_integration.py`
  Assert the new YAML parameter.

### Task 1: Add Failing Tests For Fallback Snap And Locked-Goal Policies

**Files:**
- Modify: `g1_nav/test/frontier_goal_selector_test.cpp`
- Modify: `g1_nav/test/frontier_navigation_result_policy_test.cpp`

- [ ] **Step 1: Write the failing selector test for the fallback ring**

```cpp
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
  EXPECT_DOUBLE_EQ(selected->goal_xy.first, 1.5);
  EXPECT_DOUBLE_EQ(selected->goal_xy.second, 3.5);
}
```

- [ ] **Step 2: Write the failing policy tests for locked-goal navigation and retry-once**

```cpp
TEST(FrontierNavigationResultPolicy, DoesNotPreemptActiveFrontierNavigation)
{
  EXPECT_FALSE(g1_nav::should_accept_new_frontier_goal_while_navigating(true));
  EXPECT_TRUE(g1_nav::should_accept_new_frontier_goal_while_navigating(false));
}

TEST(FrontierNavigationResultPolicy, RetriesSameFrontierOnlyOnceWhenRetryGoalExists)
{
  EXPECT_TRUE(g1_nav::should_retry_same_frontier_after_failure(true, false, true));
  EXPECT_FALSE(g1_nav::should_retry_same_frontier_after_failure(true, true, true));
  EXPECT_FALSE(g1_nav::should_retry_same_frontier_after_failure(false, false, true));
  EXPECT_FALSE(g1_nav::should_retry_same_frontier_after_failure(true, false, false));
}
```

- [ ] **Step 3: Run the gtests to verify RED**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R 'frontier_goal_selector_test|frontier_navigation_result_policy_test'`

Expected: FAIL because `fallback_snap_radius`, `should_accept_new_frontier_goal_while_navigating()`, and `should_retry_same_frontier_after_failure()` do not exist yet.

### Task 2: Implement Minimal Selector And Policy Changes

**Files:**
- Modify: `g1_nav/include/g1_nav/frontier_goal_selector.hpp`
- Modify: `g1_nav/src/frontier_goal_selector.cpp`
- Modify: `g1_nav/include/g1_nav/frontier_navigation_result_policy.hpp`

- [ ] **Step 1: Add the new selector config field and policy helpers**

```cpp
struct FrontierGoalSelectorConfig
{
  double snap_radius{1.0};
  double fallback_snap_radius{1.0};
  double goal_clearance_radius{0.35};
};

inline bool should_accept_new_frontier_goal_while_navigating(bool navigating)
{
  return !navigating;
}

inline bool should_retry_same_frontier_after_failure(
  bool has_current_frontier,
  bool retry_used,
  bool has_retry_goal)
{
  return has_current_frontier && !retry_used && has_retry_goal;
}
```

- [ ] **Step 2: Update the selector to try the fallback ring after the primary ring**

```cpp
for (const double radius : {config.snap_radius, config.fallback_snap_radius}) {
  if (radius <= 0.0) {
    continue;
  }
  auto candidates = ring_candidates(global_costmap, anchor_xy, radius);
  std::sort(candidates.begin(), candidates.end(), ...);
  for (const auto & candidate_xy : candidates) {
    if (is_goal_admissible(grid_map, global_costmap, candidate_xy, config.goal_clearance_radius)) {
      return FrontierTarget{frontier, anchor_xy, candidate_xy};
    }
  }
}
```

- [ ] **Step 3: Run the gtests to verify GREEN**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R 'frontier_goal_selector_test|frontier_navigation_result_policy_test'`

Expected: PASS for both gtest targets.

### Task 3: Wire The Explorer To Lock Goals, Retry Once, And Expose The YAML Parameter

**Files:**
- Modify: `g1_nav/src/frontier_explorer.cpp`
- Modify: `g1_nav/config/frontier_explorer_params.yaml`
- Modify: `g1_nav/test/test_realsense_nav2_integration.py`

- [ ] **Step 1: Add explorer state for fallback radius and one-shot frontier retry**

```cpp
declare_parameter("frontier_fallback_snap_radius", 1.0);
frontier_fallback_snap_radius_ = std::max(
  frontier_snap_radius_, get_parameter("frontier_fallback_snap_radius").as_double());

bool current_frontier_retry_used_{false};
```

- [ ] **Step 2: Keep the active goal locked while Nav2 is running**

```cpp
if (navigating_) {
  return;
}
```

Place this after the frontier visualization update and before any new target is dispatched so refresh cycles do not preempt or blacklist the active navigation task.

- [ ] **Step 3: Retry the same frontier once on abort or timeout cancellation**

```cpp
std::optional<FrontierTarget> retry_target = build_retry_target_for_current_frontier();
if (g1_nav::should_retry_same_frontier_after_failure(
      current_frontier_.has_value(), current_frontier_retry_used_, retry_target.has_value()))
{
  clear_navigation_state();
  current_frontier_retry_used_ = true;
  current_frontier_ = retry_target->anchor_xy;
  navigate_to(retry_target->goal_xy.first, retry_target->goal_xy.second, ...);
  return;
}

blacklist_current_frontier("Navigation aborted by Nav2");
```

- [ ] **Step 4: Stop blacklisting immediately on progress timeout**

```cpp
progress_timeout_cancel_requested_ = true;
cancel_current_goal();
```

Let the canceled-result handler decide whether to retry or blacklist.

- [ ] **Step 5: Add and verify the YAML parameter**

```yaml
frontier_fallback_snap_radius: 1.0
```

Update the integration assertion:

```python
assert params['frontier_snap_radius'] == 0.5
assert params['frontier_fallback_snap_radius'] == 1.0
assert params['goal_clearance_radius'] == 0.1
```

- [ ] **Step 6: Run targeted verification**

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R 'frontier_goal_selector_test|frontier_navigation_result_policy_test'`

Expected: PASS

Run: `cd /home/unitree/ros2_ws && colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R realsense_nav2_integration_test`

Expected: PASS
