# Frontier Debug Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show raw frontier cells and raw frontier centroids in RViz alongside the existing snapped navigation goals.

**Architecture:** Extend `FrontierSearch` to preserve frontier member cells, then reuse those real search results inside `frontier_explorer` marker publication. Do not add a second search path or a custom debug message.

**Tech Stack:** ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, C++17, RViz MarkerArray

---

## File Structure

- `include/g1_nav/exploration_types.hpp`
  Add frontier member-cell storage.
- `src/frontier_search.cpp`
  Preserve member cells while building frontier clusters.
- `test/frontier_search_test.cpp`
  Add failing test coverage for frontier member-cell storage.
- `src/frontier_explorer.cpp`
  Publish raw frontier cell and centroid markers from `observed_frontiers`, while keeping snapped goal markers.

### Task 1: Add Failing Frontier-Cell Test

**Files:**
- Modify: `test/frontier_search_test.cpp`

- [ ] **Step 1: Add a failing test that expects frontier member cells to be preserved**
- [ ] **Step 2: Run `colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R frontier_search_test` and confirm the new assertion fails**

### Task 2: Preserve Frontier Cells In Search Results

**Files:**
- Modify: `include/g1_nav/exploration_types.hpp`
- Modify: `src/frontier_search.cpp`
- Test: `test/frontier_search_test.cpp`

- [ ] **Step 1: Add member-cell storage to `g1_nav::Frontier`**
- [ ] **Step 2: Populate the member cells inside `build_frontier()`**
- [ ] **Step 3: Re-run `frontier_search_test` and confirm it passes**

### Task 3: Publish Raw Frontier Cells And Centroids

**Files:**
- Modify: `src/frontier_explorer.cpp`

- [ ] **Step 1: Change marker publication to accept raw `observed_frontiers` plus filtered `targets`**
- [ ] **Step 2: Publish `frontier_cells` as a `POINTS` marker built from raw frontier member cells**
- [ ] **Step 3: Publish `frontier_centroids` as a `SPHERE_LIST` marker built from raw frontier centroids**
- [ ] **Step 4: Keep snapped `frontier_goals` markers for filtered targets**

### Task 4: Verify End-To-End

**Files:**
- Verify only

- [ ] **Step 1: Run `colcon build --packages-select g1_nav`**
- [ ] **Step 2: Run `colcon test --packages-select g1_nav --event-handlers console_direct+`**
- [ ] **Step 3: Run `colcon test-result --verbose`**
