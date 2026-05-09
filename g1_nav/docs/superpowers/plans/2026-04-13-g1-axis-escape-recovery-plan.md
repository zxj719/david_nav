# G1 Axis Escape Recovery Implementation Plan

**Goal:** Disable lateral velocity during normal navigation, add an embedded-obstacle recovery that
escapes along only one axis, and make frontier failure handling retry once before blacklisting.

**Architecture:** First lock the expected contract with failing tests. Then add a custom recovery
behavior plus BT wrapper in `g1_nav`, wire it into both embedded-recovery trees, clamp normal
tracking `vy` to zero, and finally update frontier failure policy so timeout / abort first retry
the same frontier before blacklisting.

**Tech Stack:** ROS 2 Humble, Nav2 behavior plugins, BehaviorTree.CPP, `ament_cmake`,
`ament_cmake_gtest`, `ament_cmake_pytest`, C++17, Python `pytest`

## Task 1: Add Failing Tests

**Files:**

- Modify: `g1_nav/test/frontier_navigation_result_policy_test.cpp`
- Modify: `g1_nav/test/test_realsense_nav2_integration.py`

- [ ] Add policy tests that require no immediate blacklist on progress-timeout cancellation and
      one-shot retry before blacklisting.
- [ ] Add integration assertions for:
  - zero `max_vel_y`
  - single `vy_samples`
  - zeroed velocity smoother `y` limits
  - new BT plugin library
  - axis-escape action presence in both embedded-recovery XMLs

## Task 2: Implement Minimal Policy And Config Changes

**Files:**

- Modify: `g1_nav/include/g1_nav/frontier_navigation_result_policy.hpp`
- Modify: `g1_nav/src/frontier_explorer.cpp`
- Modify: `g1_nav/config/nav2_params_2d.yaml`

- [ ] Clamp normal tracking to differential-style motion.
- [ ] Add pure helpers for retry-once and timeout-blacklist deferral.
- [ ] Update `frontier_explorer` so progress timeout cancels first, then retries or blacklists in
      the result callback.

## Task 3: Add Axis Escape Behavior And BT Wrapper

**Files:**

- Create: `g1_nav/include/g1_nav/axis_escape_behavior.hpp`
- Create: `g1_nav/src/axis_escape_behavior.cpp`
- Create: `g1_nav/include/g1_nav/axis_escape_recovery_action.hpp`
- Create: `g1_nav/src/axis_escape_recovery_action.cpp`
- Modify: `g1_nav/CMakeLists.txt`
- Modify: `g1_nav/package.xml`
- Modify: `g1_nav/config/nav2_params_2d.yaml`

- [ ] Implement a custom behavior plugin that reads local costmap + footprint, infers the dominant
      obstacle side, and commands only one escape axis until clear.
- [ ] Implement a BT action wrapper that calls the custom behavior from Nav2 BT XML.
- [ ] Register both shared libraries and add behavior-server plugin configuration.

## Task 4: Wire Behavior Trees

**Files:**

- Modify: `g1_nav/behavior_trees/navigate_to_pose_w_g1_embedded_recovery.xml`
- Modify: `g1_nav/behavior_trees/navigate_through_poses_w_g1_embedded_recovery.xml`

- [ ] Replace the embedded `BackUp` stage with the axis-escape action.
- [ ] Keep the robot inside the embedded recovery subtree until the embedded condition clears or
      the escape action fails.

## Task 5: Verify

- [ ] Run targeted gtests for frontier policy.
- [ ] Run `pytest` integration checks for launch / config / BT assertions.
- [ ] Run a package build for `g1_nav` to validate plugin symbols and includes.
