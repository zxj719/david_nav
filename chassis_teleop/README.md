# chassis_teleop

The package provides a keyboard state machine, owned TTY reader and standalone
`chassis_teleop` executable. `navrun up --mode teleop` now provides recorded combined
startup with driver, adapter and a separately supervised keyboard. The convenience
`keyboard_teleop.launch.py` delegates to that same recorded owner, not three nodes
started concurrently. It does not bypass the lock, publisher guard or TTY backup.
This is not yet an operator-accepted replacement for alias 5k.

After loading Humble, the refactor overlay and the shared ROS/DDS environment,
operators can preview the entry without starting ROS or touching hardware:

```bash
ros2 launch chassis_teleop keyboard_teleop.launch.py robot:=david dry_run:=true
```

`robot` defaults to `david`; `start_driver` accepts auto/on/off (default auto).
`layout` defaults to wasd and also accepts uiojkl. Empty `linear_speed` (m/s) and
`angular_speed` (rad/s) retain profile gears. Optional `record` (light/full/off),
`repo`, `runs_root` and `source_root` map directly to navrun's selectors. A runtime
launch is an active host operation, not a diagnostic; production deployment and
physical motion acceptance remain manual tasks.

Direct `navrun up --mode teleop` accepts `--layout`, `--linear-speed` and
`--angular-speed`; unlike the convenience launch it retains profile layout when
omitted. Overrides are teleop-only. Explicit gears must be finite, nonnegative
and no higher than either the keyboard ceiling or adapter hard limit. They do
not raise ceilings or disable deadzone lifting: David's nonzero gears below the
profile floor are still raised to that floor. Zero stays zero. Source profile and
mode snapshots remain byte-for-byte intact; provided selectors are separately
frozen in `params/inputs/teleop_options.yaml`, and rendered parameters/runtime
bindings use those same frozen values. Hardware device paths, topics, deadzones
and safety ceilings cannot be overridden through these keyboard selectors.

The outer launch exits after navrun finalization. Normal q returns zero; navrun
failures raise a launch error (outer exit 1), with the exact status in the run
manifest. Ctrl-C similarly preserves navrun's 130 in the record. Shutdown also
explicitly forwards SIGINT to the owned navrun action: it does not rely on
Humble's interactive assumption that a terminal signal already reached children.
Both private-terminal Ctrl-C and a signal sent only to launch are tested. Launch allows
60 seconds before SIGTERM escalation and another 60 before SIGKILL to accommodate
owned-group cleanup and finalization. Forced termination of the supervisor still
cannot guarantee hardware stopping; no software-only test makes that claim.

Standalone startup acquires the navrun lock, opens the configured TTY and observes
all configured command topics (including its own output) for `discovery_sec` before
creating a publisher. Any existing publisher rejects startup, even one named like
an adapter. Rejection publishes nothing. Discovery errors fail closed, but DDS
discovery is not an atomic barrier against uncoordinated processes starting later.
INT/TERM, quit and terminal failures stop an armed output before restoring terminal
attributes, destroying ROS and releasing the lock. An already running adapter is
not exempted; combined startup performs the keyboard check before starting its adapter.

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
Navrun now supplies this parent-side ordering and terminal backup. A missing TTY
fails before platform steps. The prepared graph records supervisor ownership;
launch omits the keyboard rather than trying to pass capabilities through launch.
Normal q finishes the recording with status zero; a killed keyboard fails the run
and triggers chassis cleanup followed by parent-side terminal restoration.
The lease is retained until all groups drain, including any cleanup retries.
Teleop refuses preflight cleanup targets (including a legacy navigation stack
without the new lock) before host operations. Its keyboard hard-limit checks do
not require maps, localization or Nav2 inputs.

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
The installed convenience entry is also exercised through real LaunchService,
using a private controlling PTY and fake-profile/host fixtures; normal quit,
interruption, publisher conflict and missing TTY are checked without motion.
Actual SSH/launch controlling-terminal behavior remains P4.U2 operator acceptance.
The installed `teleop_twist_keyboard` package is an emergency operator alternative;
its speed/topic wiring still requires an explicit safety review before use.
