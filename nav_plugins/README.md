# nav_plugins

Nav2 behavior-tree recovery plugins used by the single navigation pipeline.
This package has no chassis, launch, task-management, sensor bridge or costmap
layer entry point.

`EscapeObstacle` and `RobotEmbeddedInObstacle` keep their established BT
registration names. Robot values are rendered into the templates under
`behavior_trees/` through `config/bt_bindings.yaml`; the plugins contain no
robot-specific frame default.

Build and test in the isolated refactor workspace:

```sh
colcon build --symlink-install --packages-select nav_plugins
ROS_DOMAIN_ID=99 colcon test --packages-select nav_plugins
colcon test-result --verbose
```

The tests exercise the recovery behavior and frame resolution without opening
hardware devices or sending velocity commands.
