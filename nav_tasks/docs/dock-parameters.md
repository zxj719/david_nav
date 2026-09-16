# Dock parameter compatibility bridge

`api.dock_config_from_parameters(mapping)` validates the complete old sixteen
ROS parameters and returns immutable `DockNodeConfig(goal_topic, action_name,
runtime)`. The runtime member is the existing `DockRuntimeConfig` accepted by
`create_runtime`. No ROS/YAML imports, node, client, subscriptions or files are
needed by this conversion. All names and policy values are explicit: the binder
does not select David frames, geometry or precise-mode tuning.

The existing navrun precise renderer produces the accepted mapping at
`dock_navigator.ros__parameters`. Its file is accepted by the executor's new
`--dock-config-file` option. Keep its footprint as the existing string-valued
ROS parameter; it is parsed into immutable float pairs only inside the binder.
Endpoint/frame strings are not trimmed or rewritten; whitespace-only values
are rejected. Nonfinite/negative policy values, noninteger correction counts,
thresholds outside 0–100, invalid polygons and missing/unknown keys fail early.
This stricter malformed-configuration handling does not alter valid old values.

`api.declare_dock_parameters(node, defaults)` first validates defaults, then
declares all sixteen names in their old order, reads effective ROS values and
validates again. Existing launch/ROS CLI parameter overrides therefore continue
to work without turning a POI capture frame into the Dock navigation frame.
The adapter does not rename the supplied node or claim ownership of it. On
partial declaration/read failure, the outer owner must retain and clean that
node. No motion resource is created here. Spin endpoint (`spin`) and pose-read
timeout (2 seconds) are explicit optional API arguments, not new ROS parameters.

The canonical `tasks.dock.*` schema entries document this flat ROS contract.
They have no implicit defaults and do not automatically turn a profile section
into runtime policy. The existing renderer and its mode overlay remain the
source of concrete robot values; the `executor.dock_config_file` schema describes
the actual new file selector. Profile-native policy migration can be a separate
change, without silently changing old launch behavior.

This is a configuration bridge, not a replacement Dock node yet. The outer
console still rejects explicit Dock configuration until its ROS ingress and
multithreaded host are integrated. `/goal_pose` remains accepted when explicitly
supplied; the R8 standalone-default correction is a separate red/green task.
Before replacing the old node, freeze the declaration-name reference used by
`test_dock_parameters.py` together with the existing workflow/action log goldens.

Tests compare the declaration surface, ROS overrides, complete David/example
precise render output, endpoint precedence and the file→public factory→worker
chain using fake ROS I/O only. These are not physical motion/settling evidence.
