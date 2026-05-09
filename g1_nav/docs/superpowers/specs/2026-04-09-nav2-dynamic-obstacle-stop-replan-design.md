# 2026-04-09 Nav2 Dynamic Obstacle Stop and Replan Design

## Summary

This change hardens runtime navigation against newly appearing obstacles without changing the
frontier-selection policy.

The selected approach is:

- insert `nav2_collision_monitor` after Nav2's velocity smoother so forward obstacles can stop
  motion immediately
- add `/scan` obstacle marking to `global_costmap` so the planner can replan around dynamic
  obstacles instead of following stale static-map paths
- strengthen DWB's obstacle critic so local control gives more weight to avoidance when a path
  becomes risky

## Problem

The current stack can keep executing toward a valid frontier goal even after a new obstacle
appears on the path.

Two concrete gaps cause that behavior:

1. `global_costmap` only consumes the SLAM static map, so dynamic obstacles from `/scan` do not
   trigger replanning.
2. The current behavior-tree guard only reacts after the robot footprint overlaps lethal cells in
   the local costmap, which is too late for a clean stop.

As a result, the robot may continue publishing forward motion while the path is already blocked.

## Design

### Collision Monitor

Launch `nav2_collision_monitor` as an always-on runtime safety layer between `velocity_smoother`
and `g1_move`.

It will:

- subscribe to smoothed Nav2 commands on `cmd_vel`
- observe `/scan`
- publish gated commands on `cmd_vel_safe`

The monitor uses two forward-facing polygons:

- a larger slowdown zone to reduce speed before contact
- a tighter stop zone to clamp commands to zero when an obstacle is directly ahead

This is intentionally independent from frontier logic and from BT recovery timing.

### Global Dynamic Replanning

Extend `global_costmap` with a scan-based `ObstacleLayer`.

That keeps dynamic obstacles visible to `ComputePathToPose`, so the 1 Hz replanning loop in the
NavigateToPose tree can produce a new path once the obstacle is observed in map coordinates.

### Local Controller Bias

Increase the DWB `BaseObstacle` critic weight so the controller is less willing to preserve path
tracking when obstacle cost rises near the current trajectory.

This is a supporting adjustment, not the primary safety mechanism.

## Scope

This change modifies:

- launch wiring
- Nav2 runtime configuration
- integration tests that lock the launch and config contract

It does not modify:

- frontier-goal locking or preemption policy
- `g1_move` deadman timeout handling
- the embedded-obstacle BT node

## Risks and Tradeoffs

- `collision_monitor` can make behavior more conservative and less smooth in tight indoor spaces.
- If `/scan` quality is noisy, the slowdown/stop polygons may need runtime tuning to avoid false
  positives.
- Adding scan obstacles to `global_costmap` may reduce path availability in narrow passages, but
  that is preferable to continuing into a blocked path.

## Verification

Add or update tests to assert:

- the bringup launch starts `collision_monitor`
- `g1_move` consumes `cmd_vel_safe`
- `global_costmap` now includes scan obstacle marking
- the collision-monitor config uses `/scan` and the safe output topic
- the DWB obstacle critic weight is raised

Runtime success criteria:

- the robot stops before footprint contact when a new obstacle appears ahead
- Nav2 replans around dynamic obstacles once they are visible in the global costmap
- existing `g1_nav` build and integration tests remain green
