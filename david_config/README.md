# david_config

Configuration data of the David robot. This package holds no code: robot
specific values live here so that navigation packages stay robot independent
(refactor plan §5.3, §12). Porting to another robot means copying this package
to `<robot>_config` and changing values, not code.

## Status

| Component | State | Plan reference |
|---|---|---|
| `robot/david.yaml`: robot profile (frames, footprint, limits, chassis adapter and driver, teleop, localization, sensors, platform, preflight, recording) with the values production runs today | available (PROGRESS P2.2a) | plan §12.2 |
| `modes/{tour,precise,bench_slamgt,teleop}.yaml` overlays, `sites/daolan/`, URDF | planned (P2.2b) | plan §12.2, §14 |

## Frames

`base_link` and `base_footprint` are different frames and are never merged:
Nav2, docking and costmaps use `frames.nav_base` (`base_footprint`); the body
frame published by localization is `frames.body` (`base_link`).

## How the values are guarded

`test/test_robot_profile.py` compares the profile with the effective parameters
dumped from the pre-refactor bringup (`david_nav/test/golden/params`, PROGRESS
P0.7), the `david_move` and keyboard defaults, the point-cloud-to-scan settings
of the bringup launch and the wheeltec driver launch defaults. It also checks
the profile against every package schema found in the workspace (`navcfg`).
Until Nav2 parameters are rendered from the profile (P2.3, P2.4), a changed
value fails these tests instead of silently diverging from production.

## Testing

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --metas src/colcon.meta --packages-up-to david_config
ROS_DOMAIN_ID=99 colcon test --packages-select david_config --event-handlers console_direct+
colcon test-result --verbose
```
