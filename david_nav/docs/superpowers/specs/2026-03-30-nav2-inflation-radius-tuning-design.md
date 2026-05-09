# 2026-03-30 Nav2 Inflation Radius Tuning Design

## Summary

This change tunes Nav2 inflation radii to reduce probabilistic `Navigation aborted by Nav2`
failures that happen when the robot follows an otherwise reasonable global path near
inflated obstacles in narrow spaces.

The key adjustment is to make the global planner more conservative than the local controller:

- `global_costmap.inflation_layer.inflation_radius = 0.35`
- `local_costmap.inflation_layer.inflation_radius = 0.20`

The robot footprint remains circular with `robot_radius = 0.35`.

## Problem

Current runtime behavior shows this pattern:

1. The global path looks acceptable in RViz.
2. As the robot approaches an obstacle boundary, local execution starts to oscillate or stall.
3. Nav2 eventually aborts, especially in tighter spaces.

The current configuration uses `robot_radius = 0.35` but both costmaps still use
`inflation_radius = 0.25`. That makes the global planner less conservative than the robot
geometry suggests, while the local controller still has to execute near inflated obstacle
boundaries.

## Design

### Global Costmap

Set `global_costmap` inflation radius equal to the robot radius.

Rationale:

- Encourage the global planner to choose paths that stay farther from walls and obstacle edges.
- Better align the global cost field with the robot's circular footprint size.
- Reduce cases where a path is technically valid but too close to obstacles for reliable local
  execution.

### Local Costmap

Set `local_costmap` inflation radius smaller than the global value.

Rationale:

- Keep local control from becoming overly conservative in narrow passages.
- Reduce the chance that near-edge execution is treated as effectively embedded or infeasible
  during short-lived map or sensor fluctuations.
- Preserve some room for the local controller to track a global path that passes through tighter
  geometry.

### Scope

This change only tunes inflation radii and the config-level assertions around them.

It does not change:

- controller critics
- progress checker thresholds
- recovery behavior tree logic
- frontier-goal clearance logic

## Chosen Values

- `robot_radius = 0.35`
- `global inflation_radius = 0.35`
- `local inflation_radius = 0.20`

These values intentionally make the planner more conservative than the executor.

## Risks and Tradeoffs

- Some very narrow passages that were previously planned may no longer receive a global path.
- Local behavior may still abort in edge cases if the dominant cause is DWB critic weighting or
  progress timeout rather than inflation alone.
- A smaller local inflation radius improves passability but also makes local control less
  conservative near obstacles, so this should be evaluated in real runs.

## Verification

Add or update configuration tests to assert:

- the global inflation radius matches the robot radius
- the local inflation radius is smaller than the global inflation radius
- the circular footprint configuration remains intact

Runtime success criteria:

- fewer `Navigation aborted by Nav2` failures when tracking near walls
- less oscillation or stall behavior in narrow passages
- no regression in existing config and unit tests
