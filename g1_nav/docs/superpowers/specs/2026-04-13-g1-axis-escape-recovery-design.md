# 2026-04-13 G1 Axis Escape Recovery Design

## Summary

This change makes normal Nav2 tracking behave like a differential model on G1 while adding a
dedicated embedded-obstacle recovery that can escape along a single axis.

The selected approach is:

- disable `y` velocity during normal Nav2 path tracking and smoothing
- keep embedded-obstacle handling inside the Nav2 behavior-tree recovery path
- add a custom recovery behavior that inspects the local costmap, estimates the dominant obstacle
  side around the robot footprint, and commands only one escape axis at a time
- stop immediate frontier blacklisting on the first abort or progress-timeout cancellation so the
  same frontier can be retried once before it is treated as unreachable

## Problem

Two conflicting motion requirements exist in the current stack:

1. During normal navigation, G1 should not use lateral `y` velocity. The current DWB and velocity
   smoother configuration still allow non-zero `vy`.
2. When the robot footprint is already embedded in costmap obstacles, a fixed `BackUp` action is
   too narrow. The robot should move opposite the dominant obstacle direction, which may require
   either `x` or `y` escape motion.

In parallel, frontier handling is still too aggressive about blacklisting goals after transient
Nav2 aborts and progress cancellations, which mixes embedded-recovery cases, dynamic obstacles,
and truly unreachable frontiers.

## Design

### Differential-Style Tracking

Normal path tracking remains forward and yaw-only:

- `FollowPath.max_vel_y` becomes `0.0`
- `FollowPath.vy_samples` becomes `1`
- `velocity_smoother.max_velocity[1]` and `min_velocity[1]` both become `0.0`

This keeps regular tracking consistent with a simplified differential model while preserving
existing recovery and safety layers.

### Embedded Recovery Behavior

Add a custom Nav2 behavior plugin plus BT action wrapper in `g1_nav`.

The behavior will:

- subscribe to the same local costmap and footprint topics already used by the embedded detector
- classify lethal overlap around the footprint into four coarse directions: front, rear, left,
  right
- choose the opposite escape direction
- publish only one linear axis at a time:
  - front obstacle dominant: negative `x`
  - rear obstacle dominant: positive `x`
  - left obstacle dominant: negative `y`
  - right obstacle dominant: positive `y`
- stay `RUNNING` while the robot is still embedded
- return `SUCCESS` only after the footprint is no longer overlapping lethal cells
- return `FAILED` only on timeout, missing data, or unrecoverable transform / costmap issues

This preserves the recovery state inside Nav2 instead of quickly surfacing a top-level
`NavigateToPose` abort.

### Behavior-Tree Wiring

Replace the embedded recovery `BackUp` stage with the new axis-escape behavior in both
NavigateToPose and NavigateThroughPoses trees.

The recovery sequence becomes:

1. brief wait
2. axis-escape recovery behavior
3. clear local costmap
4. return to planning / following

The generic recovery branch outside the embedded guard can still keep the broader clear / spin /
backup fallbacks.

### Frontier Abort Interpretation

Keep the current frontier selected through transient failures and only blacklist after retry
budget is exhausted.

Policy changes:

- do not blacklist immediately on progress timeout; cancel first and let the result handler decide
- retry the same frontier once after abort or timeout cancellation when a valid retry goal exists
- blacklist only after the retry path is already consumed or no retry goal can be built
- keep the existing handoff guard that avoids blacklisting while preempting to a new goal

This separates "temporarily blocked or still recovering" from "actually unreachable."

## Scope

This change modifies:

- Nav2 runtime parameters
- BT plugin registration and custom recovery implementation
- behavior-tree XML
- frontier retry / blacklist policy
- unit and integration tests

It does not modify:

- global frontier search methodology
- unknown-space traversal policy
- collision monitor geometry
- `g1_move` low-level timeout semantics

## Risks And Tradeoffs

- Lateral escape during recovery means the stack is no longer globally "no-y"; that exception is
  intentional and limited to embedded recovery only.
- Coarse directional classification can choose a suboptimal axis in cluttered corners, so escape
  distance and timeout must stay conservative.
- Relaxing immediate blacklisting may increase retries in environments that are truly blocked, but
  that is preferable to prematurely discarding recoverable frontiers.

## Verification

Add or update tests to assert:

- normal tracking config clamps `y` velocity to zero
- BT navigator loads the new escape behavior plugin
- both BT XMLs use the axis-escape recovery action in the embedded branch
- frontier retry policy retries once before blacklisting
- progress-timeout handling stops immediate blacklisting

Runtime success criteria:

- path tracking never publishes planned lateral motion in normal navigation
- when the robot footprint starts inside an obstacle, Nav2 remains in recovery and commands only
  one escape axis until clear
- the same frontier is retried once after transient abort / timeout before blacklisting
