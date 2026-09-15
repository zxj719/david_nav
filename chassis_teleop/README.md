# chassis_teleop (P4.4 in progress)

The package currently provides a pure keyboard state machine and an owned TTY
reader. It does **not yet install a ROS publisher executable or keyboard launch**.
Navigation/teleop mutual exclusion and ROS shutdown wiring must be integrated
before opening the runtime entry. This is not a deployable replacement for alias 5k.

`control.TeleopConfig` retains the Apache-2.0 `david_cmd.david_keyboard` standalone
defaults. `KeyboardState` owns only one command and its last motion timestamp.
Default WASD/arrows, signed deadzone lifting, ceilings, monotonic-time strict
timeout, stop and quit events are preserved. `uiojkl` also provides forward/reverse
arcs but no lateral axis, omnidirectional mode or unbounded speed increase.
Space/k generates an immediate-stop event; q/Ctrl-C generates a quit event. The
future runtime must publish the stop/burst; the pure state does not perform I/O.

The newly explicit `linear_deadzone` and `angular_deadzone` profile fields prevent
an alternate robot from inheriting David keyboard floors. Schema keys document
the node-name bindings and gear/adapter constraints. David keeps 0.20/0.35;
the fictional profile explicitly selects zero floors.

`terminal.TerminalKeyboard` opens `tty_device` (default `/dev/tty`) separately from
stdin, with NOCTTY/NONBLOCK/CLOEXEC. It saves attributes, enables cbreak mode and
restores attributes before closing, including on exceptions. Missing terminals,
regular files and disconnects report `TerminalUnavailable`, never silent success.
Only terminal device paths should be configured, never robot serial devices.

The escape parser retains fragmented sequences across bounded polls. This avoids
treating a delayed arrow suffix A as a separate WASD key. Unknown/overlong CSI
sequences are consumed as one non-motion event. Buffer size is bounded at 32 bytes,
and each poll consumes at most 256 bytes. A poll never waits beyond its requested
timeout for a missing escape suffix.

Tests compare actual old methods via AST without importing ROS, and exercise only
test-created PTY pairs. They do not read a user's terminal or send velocity messages.
Actual SSH/launch controlling-terminal behavior remains P4.U2 operator acceptance.
The installed `teleop_twist_keyboard` package is an emergency operator alternative;
its speed/topic wiring still requires an explicit safety review before use.
