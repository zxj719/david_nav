# David Embodied Navigation Workspace

This repository contains the ROS 2 source packages for the David embodied
navigation stack.

## Packages

- `david_cmd`: Wheeltec base velocity bridge and base evaluation utilities.
- `david_nav`: Nav2 bringup, frontier exploration, custom behavior-tree nodes,
  Realsense depth-to-scan integration, and the navigation executor.

## Build

Run the build from the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select david_cmd david_nav
```

After building, source the workspace overlay:

```bash
source ~/ros2_ws/install/setup.bash
```

## Common Commands

Run the base command bridge:

```bash
ros2 run david_cmd david_move
```

Launch autonomous exploration:

```bash
ros2 launch david_nav david_auto_explore.launch.py
```

Launch the headless navigation stack:

```bash
./src/david_nav/start_navigation_headless.sh
```

Record a navigation reproduction bag:

```bash
./src/david_nav/record_david_nav_repro_bag.sh
```

## Notes

Package names, Python modules, C++ namespaces, launch files, behavior tree
files, and Nav2 plugin names use the David naming consistently.
