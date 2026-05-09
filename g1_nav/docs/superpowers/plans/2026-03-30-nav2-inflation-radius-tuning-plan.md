# Nav2 Inflation Radius Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tune Nav2 inflation radii so the global planner keeps farther clearance from obstacles while the local controller remains slightly less conservative in narrow spaces.

**Architecture:** This change stays entirely at the configuration layer. First, extend the existing Python config regression test so it fails until the desired inflation policy is present. Then update the two inflation radius values and their nearby comments in the Nav2 YAML, and finish with the existing isolated `colcon` verification flow in the worktree.

**Tech Stack:** ROS 2 Humble, Nav2 YAML configuration, pytest, colcon

---

### Task 1: Lock and Apply the Inflation-Radius Policy

**Files:**
- Modify: `test/test_realsense_nav2_integration.py:177-186`
- Modify: `config/nav2_params_2d.yaml:258-323`

- [ ] **Step 1: Write the failing test**

Add an inflation-policy assertion next to the existing circular-footprint assertions in `test/test_realsense_nav2_integration.py`.

```python
def test_nav2_costmaps_use_circular_robot_radius():
    with (REPO_ROOT / 'config/nav2_params_2d.yaml').open() as stream:
        config = yaml.safe_load(stream)

    local_params = config['local_costmap']['local_costmap']['ros__parameters']
    global_params = config['global_costmap']['global_costmap']['ros__parameters']

    for params in (local_params, global_params):
        assert params['robot_radius'] == 0.35
        assert params['footprint_padding'] == 0.0
        assert 'footprint' not in params

    assert global_params['inflation_layer']['inflation_radius'] == 0.35
    assert local_params['inflation_layer']['inflation_radius'] == 0.20
    assert local_params['inflation_layer']['inflation_radius'] < (
        global_params['inflation_layer']['inflation_radius']
    )
```

- [ ] **Step 2: Run the targeted test to verify it fails**

Run:

```bash
python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/test/test_realsense_nav2_integration.py::test_nav2_costmaps_use_circular_robot_radius -q
```

Expected: FAIL because both YAML inflation radii are still `0.25`.

- [ ] **Step 3: Write the minimal configuration change**

Update the two inflation-radius values and refresh the adjacent comments in `config/nav2_params_2d.yaml`.

```yaml
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 10.0
        inflation_radius: 0.20  # Keep local control a bit less conservative in tight spaces.
```

```yaml
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 1.5
        inflation_radius: 0.35  # Match the circular robot radius for safer global paths.
```

- [ ] **Step 4: Run the targeted test to verify it passes**

Run:

```bash
python3 -m pytest /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/test/test_realsense_nav2_integration.py::test_nav2_costmaps_use_circular_robot_radius -q
```

Expected: PASS

- [ ] **Step 5: Run package verification in the isolated worktree**

Run:

```bash
colcon --log-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/log build --base-paths /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning --packages-select g1_nav --build-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/build --install-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/install --event-handlers console_direct+
colcon --log-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/log test --base-paths /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning --packages-select g1_nav --build-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/build --install-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/install --event-handlers console_direct+
colcon test-result --verbose --test-result-base /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning/.colcon/build/g1_nav/test_results
```

Expected:

- `build` exits with code 0
- `test` reports `100% tests passed, 0 tests failed out of 3`
- `test-result` reports `14 tests, 0 errors, 0 failures, 0 skipped`

- [ ] **Step 6: Commit**

Run:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning add test/test_realsense_nav2_integration.py config/nav2_params_2d.yaml
git -C /home/unitree/ros2_ws/src/g1_nav/.worktrees/nav2-inflation-radius-tuning commit -m "tune: adjust nav2 inflation radii"
```
