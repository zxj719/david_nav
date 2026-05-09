# Nav2 Dynamic Obstacle Stop and Replan Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop sooner for newly appearing obstacles and let Nav2 replan around them without relying on `g1_move` command timeouts.

**Architecture:** First lock the desired launch and config behavior with integration tests. Then add a dedicated collision-monitor config and wire it into the bringup launch so `cmd_vel` is gated into `cmd_vel_safe` before reaching `g1_move`. Finally extend `global_costmap` with scan obstacles and raise the DWB obstacle critic weight, then verify the package build and tests.

**Tech Stack:** ROS 2 Humble, Nav2 launch/config, nav2_collision_monitor, pytest, colcon

---

### Task 1: Lock the Safety Wiring in Tests

**Files:**
- Modify: `test/test_realsense_nav2_integration.py`

- [ ] Add failing assertions for:
  - `collision_monitor_params_file` launch argument
  - a `nav2_collision_monitor/collision_monitor` node in bringup
  - `g1_move` remapped from `cmd_vel` to `cmd_vel_safe`
  - a new `config/collision_monitor_params.yaml` contract
  - `global_costmap` scan obstacle layer
  - raised DWB `BaseObstacle.scale`

- [ ] Run:

```bash
python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py -q
```

Expected: FAIL because the collision-monitor file, launch wiring, and global-costmap obstacle
settings do not exist yet.

### Task 2: Add Collision Monitor Config and Launch Wiring

**Files:**
- Create: `config/collision_monitor_params.yaml`
- Modify: `launch/2d_g1_nav2_bringup.launch.py`
- Modify: `package.xml`

- [ ] Add `config/collision_monitor_params.yaml` with:
  - `cmd_vel_in_topic: cmd_vel`
  - `cmd_vel_out_topic: cmd_vel_safe`
  - forward-facing `Slowdown` and `Stop` polygons
  - `/scan` as the observation source

- [ ] Update `2d_g1_nav2_bringup.launch.py` to:
  - declare `collision_monitor_params_file`
  - launch `nav2_collision_monitor/collision_monitor`
  - launch a dedicated lifecycle manager for `collision_monitor`
  - remap `g1_move` input from `cmd_vel` to `cmd_vel_safe`

- [ ] Add the runtime dependency for `nav2_collision_monitor` in `package.xml`.

### Task 3: Replan on Dynamic Obstacles and Bias DWB Away from Risky Paths

**Files:**
- Modify: `config/nav2_params_2d.yaml`

- [ ] Extend `global_costmap.plugins` to include `obstacle_layer`.
- [ ] Add scan observation settings matching the local obstacle layer.
- [ ] Raise `controller_server.FollowPath.BaseObstacle.scale` to `0.2`.
- [ ] Keep the rest of the change minimal unless tests force more tuning.

### Task 4: Verify End to End

**Files:**
- Re-run modified files only

- [ ] Run the targeted config/launch regression suite:

```bash
env ROS_LOG_DIR=/tmp/g1_nav_ros_log python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/test/test_realsense_nav2_integration.py -q
```

Expected: PASS

- [ ] Run package build and targeted tests:

```bash
colcon --log-base /tmp/g1_nav_log build --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+
colcon --log-base /tmp/g1_nav_log test --packages-select g1_nav --build-base /tmp/g1_nav_build --install-base /tmp/g1_nav_install --event-handlers console_direct+ --ctest-args -R 'frontier_navigation_result_policy_test|frontier_goal_selector_test'
```

Expected:

- `build` exits with code `0`
- the targeted gtests pass
- the updated integration pytest stays green
