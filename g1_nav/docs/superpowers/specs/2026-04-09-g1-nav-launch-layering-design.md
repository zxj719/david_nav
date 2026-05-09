# 2026-04-09 G1 Nav Launch Layering Design

## Summary

This design separates the current navigation launch stack into three clear layers:

- `2d_g1_nav2_bringup.launch.py` becomes the reusable `slam + nav2` bringup only
- `g1_auto_explore.launch.py` adds frontier auto exploration on top of that bringup
- `realsense_depth_to_scan.launch.py` becomes a standalone local-obstacle layer that
  starts both `realsense2_camera/launch/rs_launch.py` and the
  `depthimage_to_laserscan` bridge

The desired runtime sequence is:

1. start `g1_auto_explore.launch.py`
2. start `realsense_depth_to_scan.launch.py`

This preserves the existing exploration workflow while making the Realsense obstacle
pipeline independently startable and independently debuggable.

## Problem

Current launch responsibilities are mixed:

- `2d_g1_nav2_bringup.launch.py` already contains optional frontier exploration logic
- the same bringup file also contains optional Realsense depth-to-scan inclusion
- `g1_auto_explore.launch.py` currently enables the Realsense scan bridge by default
  instead of staying focused on exploration
- `realsense_depth_to_scan.launch.py` only launches the scan bridge, so it cannot
  independently bring up the RealSense camera stack

That makes the launch hierarchy harder to reason about and does not match the desired
operational model where navigation, exploration, and Realsense local obstacle sensing
are separate layers.

## Chosen Approach

Use strict launch layering with single-purpose entry points.

### Layer 1: Nav2 Bringup

`launch/2d_g1_nav2_bringup.launch.py` remains the base launch for navigation and keeps
only behavior that belongs to generic `slam + nav2` bringup:

- Nav2 bringup include
- optional robot-state publication
- `base -> base_link` alias TF
- lidar `pointcloud_to_laserscan` bridge
- RViz
- `g1_move`

As part of this cleanup, the default RViz config filename should be renamed from
`nav2_go2_view.rviz` to `nav2_g1_view.rviz` so the package naming matches the G1 robot.

It removes these exploration- or Realsense-specific responsibilities:

- frontier explorer startup
- exploration parameter declarations
- Realsense depth-to-scan include
- Realsense bridge toggle declarations
- map warmup spin startup and declarations

The file should be usable as the clean navigation baseline without any exploration or
depth-camera assumptions.

### Layer 2: Auto Explore

`launch/g1_auto_explore.launch.py` stays as the exploration entry point and includes
`2d_g1_nav2_bringup.launch.py`.

It directly owns:

- frontier explorer startup
- map warmup spin startup
- explorer parameters
- warmup parameters
- the exploration-specific `NavigateToPose` BT XML override

It no longer declares, defaults, or forwards any Realsense-related arguments.

This makes `g1_auto_explore.launch.py` mean exactly:

- start the normal navigation stack
- then add frontier exploration behavior

It should also inherit the renamed default RViz config path
`rviz/nav2_g1_view.rviz`.

### Layer 3: Realsense Local Obstacle Layer

`launch/realsense_depth_to_scan.launch.py` becomes the standalone Realsense obstacle
pipeline and directly owns:

- including `realsense2_camera/launch/rs_launch.py`
- launching `depthimage_to_laserscan_node`
- bridging the Realsense depth topics into `/scan`

The launch keeps the current depth-to-scan parameters and adds a curated subset of
Realsense camera parameters that are sufficient for normal field startup and tuning.

Recommended initial Realsense argument surface:

- `camera_name`
- `camera_namespace`
- `serial_no`
- `config_file`
- `log_level`
- `output`
- `enable_color`
- `enable_depth`
- `align_depth.enable`
- `pointcloud.enable`
- `publish_tf`
- `tf_publish_rate`

The intent is not to mirror every parameter from `rs_launch.py`. The intent is to
expose the operationally useful subset while allowing advanced overrides later if
needed.

## Data Flow

The resulting data flow is:

- lidar point cloud -> `pointcloud_to_laserscan` -> `/scan` when lidar scan bridging is
  enabled from the navigation bringup
- Realsense depth image + camera info -> `depthimage_to_laserscan` -> `/scan` when the
  standalone Realsense launch is started
- Nav2 local costmap continues to consume `/scan`

This design intentionally keeps the Realsense launch separate in process startup while
still coupling through the existing `/scan` contract expected by Nav2.

This design does not add a scan multiplexer. The explicit operating rule is:

- when using the standalone Realsense obstacle layer, keep
  `2d_g1_nav2_bringup.launch.py` `enable_scan_bridge:=false` so Realsense is the only
  `/scan` publisher
- if operators want lidar-derived `/scan` instead, they should not start the Realsense
  launch at the same time

## Launch Semantics

### `2d_g1_nav2_bringup.launch.py`

The public arguments should cover only navigation concerns:

- `use_sim_time`
- `use_rviz`
- `rviz_config`
- `publish_robot_state`
- `params_file`
- `nav_to_pose_bt_xml`
- `nav_through_poses_bt_xml`
- `enable_scan_bridge`

No frontier, warmup, or Realsense launch arguments should remain.

### `g1_auto_explore.launch.py`

The public arguments should cover:

- all base bringup arguments that still make sense to forward
- `explorer_params_file`
- `enable_map_warmup_spin`
- `map_warmup_min_live_area_m2`
- `map_warmup_angular_speed`

It should launch the frontier explorer itself after the navigation stack is up, using
the same delayed-start pattern already present in the base bringup.

### `realsense_depth_to_scan.launch.py`

The public arguments should cover two groups:

1. camera startup arguments for the included `rs_launch.py`
2. depth-to-scan bridge arguments for `depthimage_to_laserscan_node`

The bridge defaults should stay aligned with the currently observed topic structure:

- depth image: `/camera/camera/depth/image_rect_raw`
- depth camera info: `/camera/camera/depth/camera_info`
- output scan: `/scan`

## Error Handling

The launch structure should fail in predictable ways:

- starting `g1_auto_explore.launch.py` without Realsense should still work
- starting `realsense_depth_to_scan.launch.py` without the camera connected should fail
  in the RealSense layer only
- navigation bringup should not depend on Realsense package availability
- exploration bringup should not depend on Realsense package availability

This separation is one of the main operational benefits of the redesign.

## Testing

Update `test/test_realsense_nav2_integration.py` so the launch structure is locked down
by tests.

### Bringup Expectations

`2d_g1_nav2_bringup.launch.py` should:

- still declare the base navigation arguments
- use `rviz/nav2_g1_view.rviz` as the default RViz config path
- no longer declare frontier exploration arguments
- no longer declare warmup arguments
- no longer declare Realsense bridge arguments
- no longer include `realsense_depth_to_scan.launch.py`
- no longer create the frontier explorer node

### Auto Explore Expectations

`g1_auto_explore.launch.py` should:

- include `2d_g1_nav2_bringup.launch.py`
- pass the exploration BT XML override to the base bringup
- use `rviz/nav2_g1_view.rviz` as the default RViz config path
- directly create the frontier explorer node
- directly create the warmup spin node when enabled
- not declare or forward Realsense arguments

### Realsense Expectations

`realsense_depth_to_scan.launch.py` should:

- still declare the depth-to-scan bridge arguments
- include `realsense2_camera/launch/rs_launch.py`
- pass the curated Realsense launch arguments through to that include
- directly create the `depthimage_to_laserscan_node`

## Risks and Tradeoffs

- If users were relying on `g1_auto_explore.launch.py` to implicitly start Realsense,
  they will now need to start the Realsense launch explicitly.
- If the curated Realsense argument subset is too small, a later patch may need to add
  more pass-through launch arguments.
- Because this design intentionally keeps a single `/scan` topic and does not introduce
  a mux, operators must choose one active `/scan` publisher per run.

## Verification

### Automated

- run the `g1_nav` launch integration tests that cover launch argument declarations and
  launch inclusions
- add assertions for the new launch ownership boundaries

### Runtime

1. start `g1_auto_explore.launch.py` and confirm Nav2 plus frontier exploration start
   without any Realsense dependency
2. start `realsense_depth_to_scan.launch.py` afterwards and confirm the RealSense camera
   node plus `depthimage_to_laserscan_node` both come up
3. confirm `/scan` is published from the Realsense pipeline and continues to feed local
   obstacle behavior as intended
