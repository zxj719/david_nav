# chassis_adapter

Python migration of `david_cmd.david_move` (Apache-2.0), keeping the command
algorithm and valid-profile behavior unchanged. It owns neither a serial port
nor localization, IMU, odometry or TF. The old executable remains available.

The installed executable/node name is `chassis_adapter`. Unified launch reads its
ten legacy-named ROS parameters from the run-local `adapter.yaml`; profile keys
and units are declared in `config/schema.yaml`. Defaults match the old standalone
node, while David navigation/teleop overlays retain their different limit values.
Parameters are startup settings, as in the old node; restart after changing them.

`ros2 launch chassis_adapter chassis.launch.py run_dir:=<prepared chassis run>`
uses the same validated bundle/graph as unified launch. It starts the adapter and
only an explicitly owned driver; `chassis:=off` retains just the adapter. Chassis
auto-detection remains outside launch in navrun. No map or localization is needed.
This completes the PLAN driver-plus-adapter chassis mode; the earlier intermediate
driver-only graph is replaced, without touching production alias 5.

The pipeline clamps then lifts deadzones. Raw input linear.x **or linear.y** makes
the angular command use the moving floor, even though output lateral velocity is
zero. A stationary yaw exactly at the threshold uses the stationary floor. All
unused output axes are zero. Tests compare over 3,000 commands against the actual
unchanged legacy methods, not a duplicated expected-value implementation.

Timeout uses ROS time and strict `age > timeout`, exactly as before. On entering
stale, one zero is sent, then output stays silent to release the command topic.
Startup is silent with ordinary positive timeouts. Nonpositive timeout disables
the check and retains the old periodic-output behavior. ROS clock rewinds and
unusually large timeouts are not silently reinterpreted. Configuration rejects
nonfinite numbers/negative limits; command-value nonfinite handling is not changed
in this equivalence migration and remains a separate safety-review item.

Diagnostics at 1 Hz report receipt count, input Hz, stale state, current shaped
output and limits. Rate accounting uses monotonic time without storing messages;
diagnostics never refresh command freshness or consume the one-shot stop.
INT/TERM handlers only set a stop flag. The foreground loop sends the old five
zero-message burst before destroying its node/context, then restores handlers.
Transport failure is warned, not presented as proof that hardware stopped.

Ordinary tests use in-memory publishers and never initialize ROS. Parameter and
zero-only runtime checks belong in the guarded domain-99 sandbox. Nothing here
changes production launch files or authorizes motion testing.
