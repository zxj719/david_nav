# chassis_teleop (P4.4 in progress)

The package provides a keyboard state machine, owned TTY reader and standalone
`chassis_teleop` executable. It does **not yet provide the combined keyboard launch**:
supervisor-owned lock and startup ordering remain pending. Combined teleop launch
is explicitly rejected before spawning children. This is not a replacement for alias 5k.

Standalone startup acquires the navrun lock, opens the configured TTY and observes
all configured command topics (including its own output) for `discovery_sec` before
creating a publisher. Any existing publisher rejects startup, even one named like
an adapter. Rejection publishes nothing. Discovery errors fail closed, but DDS
discovery is not an atomic barrier against uncoordinated processes starting later.
INT/TERM, quit and terminal failures stop an armed output before restoring terminal
attributes, destroying ROS and releasing the lock. An already running adapter is
not exempted; combined startup needs the pending supervisor integration.

The internal `chassis_teleop_managed` entry accepts three explicit inherited
descriptors: `--lock-fd`, `--control-fd` and `--tty-fd`, followed by ROS arguments.
It is not a public lock bypass: the lock descriptor must carry the already-held
exclusive Linux flock for the normal navigation lock path. Independently opening
that inode, using another locked file, a shared lock or an unheld descriptor is
rejected before ROS initialization. The entry closes only its own inherited
descriptors and duplicates; it never unlocks the shared open file description.

After publisher exclusion, the keyboard sends a ready byte on its inherited
Unix socket and waits up to 30 seconds for the owning supervisor's activation.
No publisher is created before activation. The owner may then start its own
adapter without a node-name exemption. Channel loss before activation publishes
nothing; after activation it stops the armed output. Liveness is checked both
before and after a key read, so a key arriving with disconnection cannot publish
motion. The supervisor must retain its lease until the keyboard and all other
owned children drain, and retain a terminal-attribute snapshot for crash recovery.
This child-side entry is implemented and tested; the combined launch's parent-side
ordering, terminal backup and lifecycle integration remain unfinished and rejected.

`control.TeleopConfig` retains the Apache-2.0 `david_cmd.david_keyboard` standalone
defaults. `KeyboardState` owns only one command and its last motion timestamp.
Default WASD/arrows, signed deadzone lifting, ceilings, monotonic-time strict
timeout, stop and quit events are preserved. `uiojkl` also provides forward/reverse
arcs but no lateral axis, omnidirectional mode or unbounded speed increase.
Space/k generates an immediate-stop event; q/Ctrl-C generates a quit event. The
runtime publishes the stop/burst; the pure state does not perform I/O.

The newly explicit `linear_deadzone` and `angular_deadzone` profile fields prevent
an alternate robot from inheriting David keyboard floors. Schema keys document
the node-name bindings and gear/adapter constraints. David keeps 0.20/0.35;
the fictional profile explicitly selects zero floors.

`terminal.TerminalKeyboard` opens `tty_device` (default `/dev/tty`) separately from
stdin, with NOCTTY/NONBLOCK/CLOEXEC. It saves attributes, enables cbreak mode and
restores attributes before closing, including on exceptions. Missing terminals,
regular files and disconnects report `TerminalUnavailable`, never silent success.
Only terminal device paths should be configured, never robot serial devices.
An inherited `descriptor` instead duplicates the supervisor-opened terminal, so a
private child session need not reopen `/dev/tty`. The original descriptor remains
owned by the caller, and terminal attributes are still restored on graceful exit.

The escape parser retains fragmented sequences across bounded polls. This avoids
treating a delayed arrow suffix A as a separate WASD key. Unknown/overlong CSI
sequences are consumed as one non-motion event. Buffer size is bounded at 32 bytes,
and each poll consumes at most 256 bytes. A poll never waits beyond its requested
timeout for a missing escape suffix.

Ordinary tests compare old methods via AST and use fake ROS infrastructure and
test-created PTY pairs. They do not initialize ROS or read a user's terminal.
Explicit sandbox checks use domain 99 and private locks/PTYs, send only stop/quit
keys or signals to owned children, and assert that every observed Twist is zero.
Actual SSH/launch controlling-terminal behavior remains P4.U2 operator acceptance.
The installed `teleop_twist_keyboard` package is an emergency operator alternative;
its speed/topic wiring still requires an explicit safety review before use.
