# 2026-04-11 G1 Embedded Obstacle Escape and Differential Nav Design

## Summary

This change makes two tightly related navigation adjustments for G1:

- remove lateral `y` velocity from normal Nav2 path tracking so the locomotion stack behaves like a differential-drive robot during navigation
- replace the current generic embedded-obstacle recovery with a directional escape behavior that keeps recovery active until the robot is no longer embedded in lethal local-costmap cells

The selected approach is:

- clamp Nav2 controller and velocity-smoother `y` motion capability to zero for normal navigation
- add one custom BT action node in `g1_nav` that estimates the dominant obstacle overlap direction from the local costmap and footprint topics, then commands a single-axis escape velocity in the opposite direction
- wire that action into the existing embedded-recovery guard in both G1 behavior trees so recovery happens before the goal is treated as failed

## Problem

Two issues are interacting today:

1. The current Nav2 configuration still allows `vy` sampling and smoothing even though the G1 locomotion path should be simplified to a differential model during autonomous navigation.
2. The current embedded-obstacle recovery only waits, clears costmaps, and backs up. That is not sufficient when the robot footprint is already overlapping obstacle cells from the side or rear, and it does not explicitly hold recovery until the robot has actually escaped the obstacle region.

Because `g1_move` promotes small planar commands to a minimum actionable magnitude, any small lateral Nav2 command can become a real side-step. Once the robot is considered embedded, the current recovery stack may choose a motion direction that does not match where the obstacle overlap actually is.

## Design

### Differential Navigation Parameters

Normal autonomous navigation should not generate lateral translation.

Update `nav2_params_2d.yaml` so that:

- `controller_server/FollowPath/max_vel_y` is `0.0`
- `controller_server/FollowPath/vy_samples` is `1`
- `velocity_smoother/max_velocity[1]` is `0.0`
- `velocity_smoother/min_velocity[1]` is `0.0`

Existing `acc_lim_y` and `decel_lim_y` values already prevent lateral acceleration and should remain `0.0`.

This intentionally applies only to Nav2-driven path tracking. The embedded escape behavior may still command a temporary `y` velocity during recovery because that motion is deliberate, short-lived, and driven by obstacle overlap rather than by path tracking.

### Directional Embedded Escape Action

Add one custom BT action plugin named `EscapeObstacle` inside `g1_nav`.

The node will:

- subscribe to the same local costmap and published footprint topics already used by `RobotEmbeddedInObstacle`
- transform the footprint into the costmap frame
- collect lethal cells that overlap the footprint polygon
- transform the overlapping cell centers into the robot base frame
- estimate the dominant overlap direction by comparing absolute pressure in `+x`, `-x`, `+y`, and `-y`
- choose one principal escape axis only
- publish the opposite-direction velocity command on `cmd_vel`

Direction-selection rule:

- front-dominant overlap -> command negative `x`
- rear-dominant overlap -> command positive `x`
- left-dominant overlap -> command negative `y`
- right-dominant overlap -> command positive `y`

Axis-selection rule:

- compare aggregate overlap magnitude on the longitudinal axis versus the lateral axis
- command only the dominant axis
- set the other linear axis and angular velocity to zero

Default escape-command rule:

- use `0.25 m/s` for longitudinal escape commands
- use `0.20 m/s` for lateral escape commands
- keep these defaults above the observed G1 linear deadband while staying within the current `g1_move` saturation limits

Lifecycle rule:

- while lethal overlap is still present, return `RUNNING` and continue publishing the chosen single-axis escape command
- after `3` consecutive non-embedded observations, publish zero velocity and return `SUCCESS`
- if required inputs are unavailable or transforms fail, publish zero velocity and return `FAILURE`
- `halt()` must always publish zero velocity

This keeps recovery active instead of briefly issuing one motion primitive and immediately falling back into the normal navigation pipeline.

### Recovery Tree Integration

Keep the current `RobotEmbeddedInObstacle` condition as the trigger and replace the first embedded-recovery stage in both:

- `navigate_to_pose_w_g1_embedded_recovery.xml`
- `navigate_through_poses_w_g1_embedded_recovery.xml`

with a directional escape sequence:

1. `CancelControl`
2. `EscapeObstacle`
3. `ClearEntireCostmap` for local costmap
4. `ClearEntireCostmap` for global costmap

The second embedded-recovery stage can remain as a fallback `BackUp`-based escape if the directional action fails or exhausts retries.

This preserves the existing recovery contract:

- embedded detection is handled before normal path-following resumes
- the navigation goal is not immediately blacklisted just because the robot needed local recovery
- recovery remains inside Nav2’s BT machinery instead of adding an external command publisher that fights the controller

### Abort / Replan Separation

This change deliberately only handles the “robot is already inside obstacle cells” case.

The “path becomes blocked while the robot is not embedded” case remains handled by the existing runtime stack:

- `nav2_collision_monitor` stops unsafe commands
- `global_costmap` scan obstacle marking exposes the blockage to the planner
- the BT replanning loop can produce a new path

Blacklist policy in `frontier_explorer` is not changed in this task.

## Files and Responsibilities

- `config/nav2_params_2d.yaml`
  - disable normal-navigation `y` velocity generation
- `include/g1_nav/escape_obstacle_action.hpp`
  - declaration for the custom BT action plugin
- `src/escape_obstacle_action.cpp`
  - directional overlap analysis, command publishing, BT action implementation
- `behavior_trees/navigate_to_pose_w_g1_embedded_recovery.xml`
  - integrate the new recovery action for single-goal navigation
- `behavior_trees/navigate_through_poses_w_g1_embedded_recovery.xml`
  - integrate the new recovery action for multi-goal navigation
- `CMakeLists.txt`
  - build and install the new BT plugin
- `test/test_realsense_nav2_integration.py`
  - assert the updated YAML and BT wiring
- `test/escape_obstacle_action_test.cpp`
  - verify directional classification and command selection

## Risks and Tradeoffs

- If local costmap data are noisy, the dominant overlap direction may oscillate. The implementation should bias toward the strongest axis and require consecutive clear observations before declaring success.
- Re-enabling temporary `y` motion during recovery means the robot is not purely differential in all states. This is intentional: the simplification applies to nominal navigation, not to short escape maneuvers.
- Publishing directly from the BT action on `cmd_vel` relies on `CancelControl` to avoid command contention with the controller server. That is acceptable because the action stays inside the Nav2 recovery subtree.
- If the robot is deeply embedded and all escape directions remain blocked, the action should fail cleanly so the next recovery stage can run rather than hanging forever.

## Testing

Add or update tests to assert:

- normal Nav2 path tracking no longer permits `y` velocity generation
- both embedded-recovery BTs invoke `CancelControl` followed by `EscapeObstacle`
- obstacle overlap in front, rear, left, and right yields the opposite single-axis escape command
- the action only returns success after consecutive clear observations
- the action publishes zero velocity on halt or failure paths

Runtime success criteria:

- during normal navigation, `cmd_vel` from Nav2 never contains lateral translation
- when the robot footprint overlaps local-costmap obstacles, recovery enters the directional escape action instead of immediately aborting
- the robot continues recovery until it exits the embedded state
- existing dynamic-obstacle stop-and-replan behavior remains unchanged
