# 2026-04-07 Scan Priority Local Costmap Design

## Summary

This design adds a custom `scan_priority_layer` to the Nav2 `local_costmap` so the
robot's front near-field can inject Realsense-derived `/scan` obstacle evidence
without using scan data to clear or lower existing map cost.

The core behavior is:

- keep `global_costmap` unchanged and continue to trust the lidar SLAM map
- keep the existing Realsense `depthimage_to_laserscan` bridge and raw `/scan`
- add a custom local costmap layer that compares each front-sector scan beam against a
  fixed ground-intersection baseline derived from camera pose and a ground plane model
- write `LETHAL_OBSTACLE` when a measured beam lands significantly closer than the
  expected ground baseline
- do not write any low or free cost when the beam matches the ground baseline; scan is
  used only as positive obstacle evidence

## Problem

Current runtime behavior shows these failure modes:

1. `local_costmap` is heavily shaped by `static_layer`, which is itself driven by the
   lidar SLAM map.
2. The Realsense-derived `/scan` can appear in RViz but does not dominate local
   traversability decisions.
3. In the front sector, the camera often sees a narrow ground line rather than a
   classic obstacle-rich planar scan.
4. The desired semantic is not "any valid scan return is an obstacle". The desired
   semantic is "a beam is obstacle evidence only when it lands meaningfully closer than
   the distance where that beam should intersect the floor".

Standard `ObstacleLayer` configuration cannot express this semantic cleanly because it
does not apply the desired ground-aware obstacle rule to the Realsense floor-line scan.

## Chosen Approach

Add a custom Nav2 costmap plugin named `scan_priority_layer` to `local_costmap`.

This layer subscribes directly to the raw `/scan` topic and applies a fixed geometric
ground model in the robot frame:

- the camera pose is read from TF
- the floor is modeled as a fixed plane in the `base` frame
- for each beam angle in a configured front-sector, compute the expected distance to the
  floor if no obstacle exists
- compare the measured distance against that expected floor distance
- if the measured distance is sufficiently earlier than the floor intersection, mark the
  corresponding near-field cells as `LETHAL_OBSTACLE`
- if the measured distance matches the floor model, do not write any traversability
  evidence into the costmap

This makes local behavior add front-sector scan-derived obstacle evidence while
preserving the lidar SLAM map everywhere else.

## Costmap Integration

The local plugin chain becomes:

- `static_layer`
- `scan_priority_layer`
- `inflation_layer`

The layer is intentionally local only.

`global_costmap` remains unchanged and continues to use the lidar SLAM map as the
global planning authority.

`scan_priority_layer` overrides only a bounded front-sector region inside the local
rolling window. It should not rewrite the entire local map.

## Ground-Aware Obstacle Rule

### Reference Frames

- geometric reasoning is performed in `base`
- the scan arrives in `camera_depth_frame`
- the plugin uses TF to transform the beam origin and directions into `base`

### Ground Model

The floor is represented as a fixed plane in `base` using a configured height
`ground_plane_z_in_base`.

For each beam angle in the active sector:

1. construct the beam ray from the camera origin in `base`
2. intersect the ray with the ground plane
3. obtain `r_ground(angle)` as the expected floor distance

### Classification

For each valid beam:

- if `r_meas < r_ground(angle) - obstacle_margin_m`, classify as obstacle
- otherwise classify as floor-compatible / non-obstacle

Additional smoothing:

- require at least `min_contiguous_beams` adjacent obstacle-classified beams before
  writing a hard obstacle segment
- treat `NaN` as unknown, not free
- only classify beams inside the configured front-sector and inside `max_range`

## Layer Write Semantics

Inside the configured front-sector and max range:

- obstacle beams cause the impacted cells to be written as `LETHAL_OBSTACLE`
- floor-compatible beams do not modify cells
- unknown beams do not modify cells

Outside the front-sector or outside the scan-priority range:

- `scan_priority_layer` does not modify cells
- `static_layer` behavior remains unchanged

This is intentionally a hard-obstacle design, not a soft bias layer.

## Parameters

Minimum parameter set:

- `enabled`
- `scan_topic`
- `sector_min_angle`
- `sector_max_angle`
- `max_range`
- `ground_plane_z_in_base`
- `obstacle_margin_m`
- `min_contiguous_beams`
- `debug_markers_enabled`

Recommended initial semantics:

- `sector_min_angle` / `sector_max_angle`
  limit the override to the forward fan where the Realsense floor-line behavior is
  meaningful
- `max_range`
  should match the near-field zone where local obstacle override is desired
- `obstacle_margin_m`
  is the main field-tuning knob; larger values make the layer more conservative
- floor-compatible beams should remain observation-only; they are used to suppress false
  obstacle classification, not to clear or lower local cost

## Failure Handling

The plugin must fail safe:

- if `/scan` is stale, do not override cells for that cycle
- if TF lookup fails, do not override cells for that cycle
- if a beam cannot produce a valid ground intersection, skip that beam
- if a beam is `NaN`, do not use it as obstacle evidence
- if no fresh scan is available, preserve existing local costmap behavior instead of
  inventing scan-derived obstacle evidence

This prevents the scan-priority logic from inventing obstacle evidence when the sensor
pipeline is degraded.

## Debugging and Observability

The plugin should optionally publish debug visualization showing:

- the active front-sector
- the expected ground baseline per beam
- beam hits classified as floor-compatible
- beam hits classified as obstacle

The raw `/scan` topic remains unchanged for RViz and field debugging.

## Scope

This design includes:

- a new custom Nav2 local costmap plugin
- local front-sector scan-priority obstacle writes only
- configuration and tests for the new plugin

This design does not include:

- changes to `global_costmap`
- DWB critic tuning
- changes to the Realsense depth-to-scan conversion algorithm
- online learning of the ground baseline
- manual per-angle lookup tables

## Risks and Tradeoffs

- A wrong ground-plane model can over-mark floor returns as obstacles.
- If the front-sector is too wide, the layer may suppress static-map caution outside the
  intended near-field cone.
- If `obstacle_margin_m` is too small, floor noise may become false obstacles.
- If `obstacle_margin_m` is too large, low obstacles may be missed.
- Because scan provides only positive evidence, it cannot cancel stale or conservative
  static-layer occupancy in front of the robot.

## Verification

### Automated

- add unit tests for beam classification against a fixed TF + ground model
- add unit tests for contiguous-beam threshold behavior
- add integration tests asserting `local_costmap` includes `scan_priority_layer`
- add integration tests asserting the new default parameters and topic names

### Runtime

- on flat floor with no obstacle, the scan-priority layer should add no new lethal cells
  to the front sector of `local_costmap`
- when an object rises into the front-sector before the expected floor intersection,
  `local_costmap` should immediately show a local hard obstacle
- outside the configured front-sector, local costmap behavior should remain dominated by
  the existing static lidar SLAM map
