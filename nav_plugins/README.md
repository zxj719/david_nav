# nav_plugins

Nav2 costmap and behavior-tree plugins extracted from `david_nav` (PLAN P3.1).
This package has no chassis or task-management entry point.

## Compatibility

The C++ namespace and public headers are `nav_plugins`. The three existing
shared-library basenames and `david_nav::ScanPriorityLayer` pluginlib lookup
alias remain available so existing Nav2 parameter files need no changes.
New pluginlib consumers may use `nav_plugins::ScanPriorityLayer`.
`EscapeObstacle` and `RobotEmbeddedInObstacle` BT registration names are unchanged.

Robot profile keys are declared by their owning package, not duplicated in this
package's schema. Layer parameters retain their current defaults during the
mechanical migration. Frame parameterization and lint are separate follow-up
commits (PLAN P3.1); the BT port default-frame exception follows PLAN §7.4.
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
