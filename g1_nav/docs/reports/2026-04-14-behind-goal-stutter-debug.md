# G1 Behind-Goal Stutter Debug Report

## Summary

This session debugged the real-robot symptom where a navigation goal behind the robot caused severe stutter or multi-second delay before motion.

The dominant issue was not a single deadband. It was a two-layer interaction:

1. Nav2 frequently chose a small forward-plus-yaw command for behind goals instead of committing to turn-in-place first.
2. The old `collision_monitor` configuration used fixed forward `Slowdown` and `Stop` polygons, so those behind-goal startup commands were repeatedly clamped to zero when the robot had only about 0.5 m free space in front.

There was also a timing amplifier:

- Realsense depth-to-scan published `/scan` in `camera_depth_frame`.
- Downstream consumers repeatedly had to resolve a timestamp-sensitive `camera_depth_frame -> base` transform.
- Under load this produced stale-source warnings and stop-go behavior.

## What Was Measured

Real-machine controlled tests used:

- `/lightning/odometry`
- `map -> base` TF
- `/cmd_vel_nav`
- `/cmd_vel_safe`

The primary motion judgement in this session was based on `/lightning/odometry` and `map -> base` TF, not `sportmodestate`.

## Baseline Evidence

### Baseline behind-position goal on minimal bringup

With the old `Stop/Slowdown` polygons:

- `nav_first ~= 0.11 s`
- `safe_first ~= 1.46 s`
- `odom_first ~= 1.97 s`
- `tf_move_first ~= 7.05 s`
- `safe_zero_while_nav_nonzero = 270`

Observed behavior:

- Nav2 started publishing almost immediately.
- `collision_monitor` repeatedly stopped the command chain.
- The robot barely made useful progress over the test window.

### Controller-only tuning was insufficient

Raising `RotateToGoal.scale` alone did not solve the real symptom.

- The controller produced less linear startup in some cases.
- But fixed forward stop polygons still blocked turning when the robot faced nearby obstacles.

## Fixes Applied

### 1. Collision monitor behavior

Changed `config/collision_monitor_params.yaml` from fixed forward `Slowdown` + `Stop` polygons to a footprint-based `Approach` polygon:

- `action_type: approach`
- `footprint_topic: /local_costmap/published_footprint`
- `time_before_collision: 1.0`
- `simulation_time_step: 0.1`

This allows turn-in-place and trajectory-aware gating instead of unconditional forward-zone full stops.

### 2. Realsense scan frame and timing

Changed `config/realsense_depth_to_scan.yaml` to:

- `output_frame: base`
- `scan_time: 0.1`

This removed the old `camera_depth_frame -> base` dependency from downstream consumers and aligned scan timing with the observed camera rate.

### 3. Collision monitor timing margins

Adjusted:

- `transform_tolerance: 0.3`
- `source_timeout: 1.0`

This reduced stale scan drops that were still visible under heavier runtime load.

## Fresh Verification

After rebuild and fresh restart with `slam2 + auto`:

- `/scan.header.frame_id == base`
- `/scan.scan_time == 0.1`
- `/collision_monitor/polygons == ['Approach']`
- `/collision_monitor/source_timeout == 1.0`
- `/collision_monitor/transform_tolerance == 0.3`

Fresh behind-pose verification on the real robot produced:

- `nav_first ~= 0.30 s`
- `safe_first ~= 0.30 s`
- `odom_first ~= 0.70 s`
- `tf_rot_first ~= 1.30 s`
- `tf_move_first ~= 1.39 s`
- `safe_zero_while_nav_nonzero = 3`

This is a clear improvement over the previous state where the robot could sit for about 1 s, several seconds, or repeatedly stop before meaningful motion.

## Remaining Risk

Some `Message Filter dropping message: frame 'base' ... earlier than all the data in the transform cache` logs still appear occasionally.

They no longer reproduced the severe behind-goal full-stop symptom after the applied fix, but they remain the next place to inspect if high-CPU edge cases persist.
