# nav_plugins

Nav2 costmap and behavior-tree plugins extracted from `david_nav` (PLAN P3.1).
This package has no chassis or task-management entry point.

## Compatibility

The C++ namespace and public headers are `nav_plugins`. The three existing
shared-library basenames and `david_nav::ScanPriorityLayer` pluginlib lookup
alias remain available so existing Nav2 parameter files need no changes.
New pluginlib consumers may use `nav_plugins::ScanPriorityLayer`.
`EscapeObstacle` and `RobotEmbeddedInObstacle` BT registration names are unchanged.

When upgrading an existing symlink-install workspace, retire the old three
`install/david_nav/lib/libdavid*.so` plugin symlinks and the old
`install/david_nav/share/ament_index/resource_index/nav2_costmap_2d__pluginlib__plugin/david_nav`
resource marker before sourcing the overlay. Incremental colcon does not remove
obsolete installed artifacts, and a stale library can shadow the new package.
Back up those exact entries, or use a clean build/install destination. Never
apply this step to a live production prefix without a reviewed deployment.

Robot profile keys are declared by their owning package, not duplicated in this
package's schema. The `scan_priority_layer.robot_base_frame` ROS parameter
defaults to the parent costmap's `robot_base_frame` (bound from the profile's
`frames.nav_base`). An explicit layer override takes precedence; an empty
effective frame rejects initialization. TF lookup and debug markers use the
same effective frame. No robot frame literal remains in the scan layer.
The BT port default-frame exception follows PLAN §7.4: the legacy default is
retained only in `EscapeObstacle`; rendered XML always passes the frame.
Lateral escape policy is deliberately unchanged (TODO-E1).

## Tests

Build in the isolated refactor workspace, sourcing ROS Humble only:

```sh
colcon build --symlink-install --packages-select nav_plugins
ROS_DOMAIN_ID=99 colcon test --packages-select nav_plugins
colcon test-result --verbose
```

The C++ unit tests evaluate geometry and recovery state without starting ROS
nodes, publishing velocities, or opening hardware devices.

The separately built `scan_priority_layer_checks` executable creates ROS nodes
and checks both pluginlib names, default inheritance, explicit overrides and
empty-frame rejection. It is intentionally excluded from `colcon test`.
Run only through `bash sandbox_tests/run_sandbox_checks.sh`; that launcher
refuses an active user stack, loads the DDS whitelist, forces domain 99 and
keeps logs in a dedicated temporary directory. It never opens serial devices
or sends velocities.
