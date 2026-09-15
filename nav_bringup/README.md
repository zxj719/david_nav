# nav_bringup

Bringup, run recording and operations tooling for robot navigation stacks.
This package is robot independent: robot-specific values live in a
`<robot>_config` package.

## Status

| Component | State | Plan reference |
|---|---|---|
| `nav_bringup.lint.docstrings` | available | refactor plan §5.4, RUNBOOK §1 |
| `nav_bringup.lint.literals`: no robot frame names or robot topic prefixes in package code | available | plan §5.4 |
| `nav_bringup.navcfg.schema`: per-package `config/schema.yaml` format, catalog aggregation, value and profile checks, schema discovery | available (P2.1) | plan §12.1 |
| `nav_bringup.recorder`: run directory, manifest, `code.diff`, events, resource samples, summary, retention | available (PROGRESS P1.2a) | plan §13.2 |
| `navrun dump-params`: effective parameters of every running node into `params/effective/` (golden capture rules: helper nodes skipped, per-call timeout, retries, non-empty output; duplicate node names reported) | available (P1.2b) | plan §13.2 |
| `navrun monitor`: one node for behavior tree triggers, key topic rates and publisher counts, and map→robot TF lag (`metrics/topics.csv`, 0.2 Hz); rates, topics without publishers and TF lag percentiles in `summary.md` | available (P1.2b) | plan §10.3, §13.2 |
| Linked localization (`recorder.linked_process`; `navrun wrap --loc-process EXECUTABLE [--loc-log GLOB]`): pid, start time, command line and config SHA-256 of the separately started localization in the manifest; what its logs gained during the run copied into `logs/loc/` (last 20 MB per file); read-only | available (P1.2c) | plan §13.2 |
| Sandbox integration checks (`sandbox_tests/`): `navrun monitor` and `navrun dump-params` against live processes in DDS domain 99; not collected by `colcon test`, run with `sandbox_tests/run_sandbox_checks.sh` | available (P1.2b) | RUNBOOK §5 |
| `navrun wrap -- <command>`: record a run around an existing launch command | available (P1.3) | plan §13.1, §16.2 Phase 1 |
| Escape and recovery triggers (`recorder.bt_triggers`, `recorder.bt_log_watcher`) from `/behavior_tree_log` → `events.jsonl`, terminal line, `logs/navrun.log`, per-node counts in `summary.md`; subscribed by `navrun monitor` | available (P1.4a) | plan §10.3 |
| `navrun wrap --monitor-base-frame FRAME [--monitor-topic TOPIC]`: runs `navrun monitor` next to the command (own session; output in the terminal and `console.log`; stopped after the command; failures recorded as warnings) and dumps effective parameters once `--ready-action` (default `/navigate_to_pose`) has a server | available (P1.4b) | plan §10.3, §13.2 |
| `navrun doctor`: read-only checks (ROS/DDS environment, orphan Fast DDS segments, map yaml mode, duplicate robot_state_publisher, stale copied scripts, chassis serial device, ROS log directory size) | available (P1.5a, P1.5b) | plan §13.3 |
| `navrun doctor --live [--cmd-topic T] [--watch-topic T]`: stale ros2 CLI daemon (compared with `--no-daemon`; never started or stopped), several publishers on a command topic, publishers from more than one host (Fast DDS GID host bytes), map→odom transform missing (hint) | available (P1.5b) | plan §13.3 |
| `navrun up` | recorded navigation/chassis/teleop entry available; deployment acceptance pending (P4.2) | plan §13.1 |
| `navcfg render --profile FILE [--mode FILE] --output FILE` (`nav_bringup.navcfg.render`): a base parameter tree whose robot-specific values are `<bind:KEY>` markers, a binding table listing every parameter each profile key is written to (markers and table must agree), the robot profile and a mode overlay (profile values plus `nav2_overrides`, which may not touch bound parameters); PyYAML and the standard library only | available (P2.3) | plan §12.3 |
| `config/nav2_base.yaml` + `config/nav2_bindings.yaml`: Nav2 base parameters (the David tour values and their tuning notes) with frames, footprint, speed limits, goal tolerances and odom/scan/map topics bound to profile keys, which `config/schema.yaml` declares as required | available (P2.3) | plan §12.3 |
| `navcfg validate --profile FILE --mode FILE --site FILE --urdf FILE --bt-dir DIR [--source-root SRC]`: schema checks plus the frame, speed, precise-mode, trinary-map, teleop, diff-drive BT and POI-frame rules; warnings do not reject the configuration | available (P2.5) | plan §12.4 |
| `navcfg doc` | planned (P2.6) | plan §12.1 |
| `nav.launch.py` | planned (P4.1) | plan §7.1 |

## Layout

```
nav_bringup/
├── nav_bringup/
│   ├── __init__.py
│   └── lint/
│       └── docstrings.py   # Google-style Args/Returns/Yields/Raises checks
├── test/
│   ├── test_docstring_policy.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Code style

New navigation packages follow the Google Python Style Guide:

* `ament_flake8` with the ROS default configuration (99 columns, Google import order).
* `ament_pep257 --convention google` (summary on the first docstring line).
* `nav_bringup.lint.docstrings.check_paths` in a unit test, for the sections that
  pep257 does not check.

To apply the docstring policy to another package, add a test:

```python
from pathlib import Path

from nav_bringup.lint import docstrings


def test_docstring_policy():
    """Public API documents its arguments, returns and raised exceptions."""
    package_dir = Path(__file__).resolve().parents[1] / 'my_package'
    violations = docstrings.check_paths([package_dir])
    assert not violations, docstrings.format_violations(violations)
```

## Testing

### Prepared launch entry (P4.1)

`nav.launch.py` consumes a pre-rendered run directory. It does not render, copy,
remove, probe or signal external processes, and never starts localization.
The selector arguments are `robot`, `site`, `mode`, `chassis:=on|off`,
`executor:=true|false`, `composition:=false` and required `run_dir`.
Robot/site/mode selectors must match the prepared bundle. Composition remains
explicitly rejected until P5.4, rather than silently ignored.

`launch.json` has version 1 and fields `schema_version`, `robot`, `site`, `options`.
Options are the fields of `launching.graph.GraphOptions`, except `run_dir` comes
from the launch argument; tuple-valued executor arguments are JSON arrays.
All selected parameters must already exist under `params/rendered/`:
`driver.yaml`, `adapter.yaml`, `nav2.yaml`, `lifecycle.yaml`, and the applicable
`map_server.yaml`, `robot_state.yaml`, `scan_bridge.yaml`, `dock.yaml`,
`rviz.yaml` or `teleop.yaml`. RViz also uses `inputs/navigation.rviz`.
Lifecycle node order is checked against the graph before process construction.

The launch adapter eagerly resolves every executable before returning actions.
It allows only the configured number of driver respawns, shuts down on final
critical exits, and never respawns during shutdown. Each respawn uses a fresh
owned action instead of an unlimited background restart loop. External drivers
are omitted completely. The plain legacy executor CLI is protected by a
read-only action-server readiness gate and receives its exact argv, without a shell.

This is not yet an accepted production replacement for aliases: the convenience
keyboard launch and navigation/physical acceptance remain pending. Bundle preparation,
the Python chassis adapter and recorded supervision are available. No production alias has changed.
The sandbox launch checks use disposable Python children and an optional fake
Nav2 server; they never run the physical driver or submit a goal.

### Offline bundle preparation (P4.2)

`navrun.preparation.prepare_bundle(GraphOptions, BundleInputs, catalog)` renders
all five modes without creating directories, importing ROS clients, discovering
processes or opening devices. Its explicit paths work with source or installed
configuration packages. The caller still resolves chassis auto-detection and
legacy executor CLI arguments; `navrun up` supplies that orchestration.

Preparation binds driver package/executable, delay/restart budget and chassis
topics from the mode-merged profile. `chassis.driver.executable` is explicit in
both David and the fictional robot profiles. Teleop requires a matching keyboard
output and adapter input; chassis/teleop need neither map nor localization files.
The bench mode does not start a second map server. Navigation uses the existing
golden-tested Nav2/BT renderer and all offline cross-file checks. Dock receives
profile frame names, the full footprint, static map topic and the 0.05 rad/s
settle threshold. The temporary dock launch policy remains the legacy precise
policy; configurable task policy belongs to the Phase 6 migration.

The returned immutable artifacts include input snapshots, rendered node YAML,
run-local BT XML, `checks.yaml`, input hashes and `launch.json`. Map image and
manually maintained named poses are only hashed; neither is rewritten or copied.
The rendered map metadata references its absolute original image path. Recorder
manifest integration is supplied by the recorded supervisor; `fingerprints.json`
is its prepared input, not a replacement manifest.

`write_bundle` adds these files exclusively to a newly recorder-owned directory.
It rejects existing artifacts and symlinked destinations before writing, and
writes `launch.json` last. A failed write leaves diagnostic partial files without
publishing a launchable bundle; it does not overwrite or delete existing records.

### Startup preview and recorded entry (P4.2)

The installed `navrun up --dry-run` combines robot/site selection, mode overlays,
chassis auto-detection, bundle validation and executable availability. Resource
lookup uses the current ament overlay; `--source-root <workspace/src>` selects
source assets instead. Both paths use the same renderer and launch graph.
Executable validation is shared with the real launch adapter and imports no ROS
runtime. Missing executables are reported before any platform or cleanup work.

```bash
navrun up --mode tour --robot example_diffdrive --site offline --no-executor --dry-run
navrun up --mode chassis --robot example_diffdrive --dry-run
```

The JSON report contains selected processes, proposed platform commands, cleanup
PIDs, protected PIDs, warnings and missing dependencies. It never prints observed
process command lines. Dry-run does not acquire the global lock, create a run
directory, execute subprocesses, inspect/delete shared memory, or start ROS nodes.
`dry_run_preview` is a prospective path only, not a real run record.

Exit 0 means the preview is valid, not that deployment is accepted. Missing
executables/configuration return 2. Legacy executor arguments now come from the
frozen robot/site inputs; `--no-executor` still explicitly omits that process.
Actual `up` without `--dry-run` now enters the lock-scoped recorded workflow for
tour, precise, bench_slamgt, chassis and teleop. `--repo` supplies the repository recorded
in the manifest; `--source-root` also serves as the repository when `--repo` is
absent. Installed layouts without a discoverable repository require `--repo`.
`--strict` makes live-check errors stop navigation startup. Chassis never waits
for localization. The report includes exact `runtime_commands` and reports
`runtime_enabled: true` for these modes, not a claim of physical acceptance.

This is an active host operation: profile-enabled platform configuration,
identity-checked residual cleanup and conservative DDS hygiene precede launch.
Do not run it casually as a diagnostic or replace production aliases before
operator acceptance. Implementation tests replace all platform/signal operations
and the DDS directory; they never execute host cleanup on the robot.

Teleop uses a supervisor-owned keyboard and an ordered startup interlock.
The complete prepared graph records `supervisor_keyboard: true`; launch omits that
one process because navrun owns it directly. Bare teleop graphs without this
ownership selection are rejected; standalone keyboard startup retains its own
lease and publisher guard. No PID-name exemption or environment bypass exists.
The handoff uses explicit inherited descriptors: `DelegatedLock`
checks Linux fdinfo for an exclusive FLOCK record on the expected open file
description, without issuing flock or unlocking the parent. `KeyboardChannel`
provides ready/permit and EOF detection over a local socket, using per-call
nonblocking flags so it does not mutate the source descriptor's file status.
`OwnedProcess(pass_fds=...)` delegates only those explicitly selected capabilities;
its default still closes every nonstandard descriptor. Before host steps,
`KeyboardSession` opens the configured TTY in the parent's original session and
saves its attributes. Navrun starts only the keyboard first, waits for its completed
publisher check, then starts the driver/adapter launch and permits keyboard output.
No TTY or a graph conflict prevents chassis startup; rejection sends no zero.
Teleop also refuses any preflight cleanup targets, including legacy navigation
that does not hold the new lock; it cannot silently stop that stack to gain control.
Keyboard ceilings are checked against adapter hard limits even without navigation
assets. Suspicious leftovers require inspection, not a hidden cleanup exemption.
The parent holds terminal recovery state and the navigation lease through all
child cleanup retries. A killed keyboard fails the run but cannot leave cbreak
mode behind while the parent remains alive. Normal q exits with status zero;
signals and abnormal keyboard exits retain their recorded statuses.
The `chassis_teleop/keyboard_teleop.launch.py` convenience entry delegates to this
same recorded owner and propagates child failures. Its selectors are documented
in chassis_teleop's README; this does not authorize switching production 5k.
Direct up accepts teleop-only `--layout`, `--linear-speed` and `--angular-speed`.
Omitted values preserve the profile. Explicit gears reject nonfinite, negative or
over-ceiling requests; profile deadzone lifting remains unchanged. Source robot
and mode bytes/hashes are unchanged. The separate frozen
`params/inputs/teleop_options.yaml` records supplied choices and feeds rendering
and runtime bindings, so replay does not depend on mutable CLI objects or sources.

### Executor and recording preparation (P4.2)

The legacy WS executor receives `agent_server.uri`, resolved
`pois.named_poses_file` and `pois.store_file`, `executor.reconnect_delay_sec`, and
the selected navigation action. Relative POI paths are relative to the source site
directory; home paths expand but neither point file is opened or modified by this
binding step. Existing input fingerprinting still reads the named-poses file.
`pois.dock_poi_ids` in site.yaml is an optional list used **only in precise mode**;
David's site retains POI_017, while tour keeps ordinary navigation. The ready gate
uses `startup.nav_ready_timeout_sec` (120 seconds for both old David entries).
Golden tests pass both argv forms through the old parser and compare every setting.
Remaining legacy frame/topic constants are Phase 6 migration work, not changed here.

The preview's `recording` entry includes enabled/reason, exact argv, run-local
`bag/data` destination and explicit selectors. Light uses `record.light_topics`;
an empty list explicitly disables recording. Full combines `record.full_topics`
and `record.full_regex` and requires at least one selection, never an implicit
`--all`. Full-only hidden/unpublished and compression options are profile-owned.
David preserves both old topic lists, regex, 100 MiB cache and zstd full recording;
large global maps remain excluded from light. Storage is SQLite, not MCAP.

Preparation does not start a recorder. Recorded startup launches it after readiness
and live checking, supervises it and finalizes it before writing summaries.
Isolated CLI tests record only an owned text
topic and verify SQLite metadata and full-mode compression without velocity traffic.

### Live configuration check and manual localization (P4.5)

`navcfg check --live --run-dir /absolute/run/record --strict` observes an existing
prepared navigation run. Source the selected workspace and its shared ROS/DDS
environment first; the checker neither changes domains/interfaces nor starts a CLI
daemon. It never launches localization, sends initial poses, submits goals, changes
parameters or publishes commands. Chassis/teleop modes are rejected because they
must not gain a localization prerequisite.

Until the configured global-to-odometry transform appears, the default is an
indefinite wait with an informational operator hint every five seconds. There is
no IMU dependency. `--wait-timeout SECONDS` optionally bounds a standalone check;
status 3 means still waiting, not successful verification or failed localization.
INT/TERM closes only the observer and returns 128+signal. The caller may explicitly
provide `--events /owned/run/events.jsonl` to append waiting/ready/check events;
without that option no run record is written.

After localization, `--sample-sec` (default 1) allows graph discovery, then a single
global `--parameter-timeout` (default 5 seconds) bounds node/TF discovery and concurrent
ListParameters/GetParameters requests. Late-discovered nodes use the remaining budget,
not an immediate absence verdict, even if TF arrives after the parameter replies.
Only declared requested names enter each value batch:
Humble C++ services otherwise reject the entire batch when one name is undeclared.
Undeclared requested names remain explicit errors, not exemptions. Checks cover
publisher counts and message types, navigation TF age,
duplicate node names and every flattened rendered parameter selected for that
node's actual parameter file. Costmap child namespaces are preserved. Dormant
selectors are warnings; an undeclared parameter on a selected node is an error,
not a silent exemption. Boolean/integer and other ROS type mismatches are errors.
Existing inert legacy parameters may therefore be reported; do not change their
values or waive findings merely to get a green report.

Errors always print. `--strict` returns 1 on errors; without it the command returns
0 after completing the diagnostic. TF age warnings retain warn-only policy.
Malformed inputs or operational failure return 2. This bounded observation is not
a continuous health watchdog and does not prove message rates, odometry quality,
hardware stop behavior or atomic graph ownership. Perception-contract sampling
is performed by the independent `nav_perception_iface/nav_interface_check` executable;
Phase 6 continuous health integration remains distinct work.

After parameter observation closes, the independent checker samples actual localization
and chassis odometry, compatible command subscribers, the odom→body→nav_base TF chain,
and optional cloud/at-stamp TF. Its single versioned report is validated against its exit
status; malformed reports, missing executables and time/output overruns are operational
errors, never silently skipped. Events include the bounded `interface_snapshot` evidence.
The child remains in the caller's process group and is joined before return, including
interruption; neither observer owns or signals localization providers.

Navigation-mode `navrun up` first invokes `--wait-localization-only` **before launch**.
This mode returns 0 only for TF readiness, not a successful full configuration check.
Manual localization waiting therefore consumes no Nav2/executor readiness budget.
After launch and readiness, the complete live check runs, with stop-on-strict-failure.
Chassis/teleop retain no localization prerequisite. Domain-99 integration checks run
the real tour/precise Nav2 graph with the example robot, fake driver and stationary
synthetic localization, without goals or a WS executor. They compare every declared
rendered value with complete effective dumps, require missing-key findings to match
those dumps, and verify strict rejection, provider independence and shutdown ownership.
These are startup/configuration checks, not driving or localization-quality acceptance.
Executor ordering and localization loss during startup remain separate integration work.

### Run-local logs and frame trace noise (P5.1)

Recorded startup sets `ROS_LOG_DIR` to the owned run's `logs/ros` for all children;
console and per-process logs remain in the same run record. The refactored Wheeltec
driver and David navigation profile default `log_tx_frames` to false. Explicitly
enabling it still emits frame traces only at DEBUG, never INFO. Memory-transport
tests cover both settings; this does not change the production driver or alias 3.

### Unit tests

`navrun.dds_hygiene` separates read-only mapping observation and candidate planning
from explicit removal. Segment and `_el` companion are one retention group: either
mapped name, an unsafe file/owner, incomplete proc maps, or changing process-directory
identities preserves the group. Initial file metadata is frozen; new files and
replacements cannot enter the cleanup generation. Each unlink refreshes mapping
and group evidence, checks the directory/file identity, and reports partial or
failed removal. Dry-run does not refresh maps or invoke removal. Tests only use
private temporary directories and fake proc maps, never actual `/dev/shm`.

The planner and cleanup require an explicit directory; there is no default
destructive CLI and no daemon stop, privilege escalation, or DDS XML modification.
Proc snapshots are not an atomic barrier against independently starting DDS
participants. Production acceptance therefore still requires a controlled startup
window and adequate mapping visibility; incomplete evidence retains files with a
warning. `navrun doctor` now shares this observation logic, reports unknown orphan
status on incomplete maps, and recognizes mapped companions consistently. It no
longer suggests starting a whole navigation stack just to clean shared memory.

`navrun.preflight.plan_preflight` consumes process snapshots and resolves chassis
auto/on/off. Residual and foreign patterns live in the robot profile; foreign
stacks are warn-only by default. Matching uses the executable/script or ROS launch
prefix, never editor arguments or inline shell text. The provider's launch branch,
its sensor/RSP siblings, higher ancestors, and external chassis branches are
protected. A shared terminal server is protected without hiding unrelated tabs.
Other users' processes are never cleanup targets.

`cleanup_preflight` sends TERM, waits at most eight seconds, then sends KILL and
confirms exit for up to 0.5 seconds. Every signal and poll uses a fresh snapshot;
new matches, changed PID identities, newly protected targets and surviving
processes reject startup instead of widening cleanup. Dry-run invokes none of
the observation, signal, clock or sleep hooks. Signal and snapshot adapters are
explicit arguments, with no implicit os.kill default.

`navrun.process_backend` provides the Linux adapters for explicit up host preparation:
read-only psutil snapshots reject incomplete views, and signaling opens a pidfd
before rechecking PID/creation time/user/argv. It has no PID-only fallback, shell,
sudo or permission change. Preview uses only the snapshot adapter; actual up can
call the signal adapter after planning and validation. Tests replace signals with fakes;
no production residual has been stopped or restarted during this implementation.

`navrun.locking.RunLock` implements the shared `/tmp/navrun.lock` lease for the
recorded `up` supervisor. Construction does no I/O. Acquire is nonblocking, checks
file ownership/type/links, and only writes diagnostic PID metadata after flock
succeeds. Release never unlinks the inode; a crashed owner's old PID does not
block a new run. The descriptor is close-on-exec, and releasing an inherited
object after fork cannot unlock the parent. Tests use private temporary paths.

`navrun.platform.plan_platform` is pure; `configure_platform(..., dry_run=True)`
never calls its runner. Enabled operations use `sudo -n --` with explicit Jetson
tool paths, DEVNULL stdin and a per-command deadline. MAXN precedes clocks;
the first failure stops the sequence and carries partial results for recording.
Already successful power settings are not rolled back or reported as undone.
`platform.maxn_mode_id` defaults to the legacy value 0, and
`platform.command_timeout_sec` defaults to 30 seconds. These are not automatically
executed by imports or the existing wrap command. Permission setup remains an
operator-owned sudoers task; the code never supplies a password or edits policy.

`navrun.startup_lifecycle.supervise` now executes an explicitly supplied startup
pipeline under an already acquired `RunLock`: owned launch → ordered readiness
and live-check gates → independent parameter dump → optional monitor/recorder → foreground
supervision → stop attempts and cleanup. Manual localization can wait indefinitely,
but launcher exits and requested signals remain observable throughout that wait.
Unexpected launcher/recorder exits fail the run, including early exit code zero;
optional monitor/dump failures only warn. Two bounded zero-command attempts bracket
launch shutdown, with the first before waiting for interrupted gates; recorder
shutdown follows the second attempt. A failed stop does not skip other cleanup.
`runtime_commands` resolves installed launch, ready, navcfg, dump, monitor, bag
and zero programs before host work; no shell or implicit daemon is used.
The outer transaction below provides manifest, signals, logs and metrics.

Each child gets an exclusive `logs/<name>.log`, run-local `ROS_LOG_DIR`, closed
stdin, and a private session. `navrun.owned_process.OwnedProcess` uses Linux
`waitid(WNOWAIT)` to retain the leader PID even after it exits, then interrupts its
group, permits flushing, kills remaining members and only reaps once the group
has drained. Repeated close never signals a reused numeric PID. Commands must not
detach descendants: Humble launch children retain their group; independently
detached helpers must be separately owned. The legacy wrap/session implementation
has not been changed by this addition. If cleanup cannot drain a group, the result
retains its live ownership handle: the outer entry point keeps the lock
and retries, rather than finalizing the run and releasing ownership. Cleanup errors and failed
zero attempts are recorded explicitly, not reported as successful stopping.

The lifecycle tests execute only fakes and private, non-ROS Python processes.
Their zero command is a text-printing placeholder, not a ROS publisher. Tests
cover early leader exit with an uncooperative grandchild, graceful log flushing,
unrelated-session survival, wait ownership loss, bounded escalation, gate failure,
manual-wait interruption, recording failure, event-file errors and cleanup retry.

`navrun.recorded_startup.run_recorded_startup` connects that lifecycle to the run
recorder under one `RunLock`. The explicit repository is fingerprinted, a new
manifest is written before configuration preparation, the prepared bundle and
input hashes are persisted, all supplied commands are validated, and only then
are the explicitly supplied platform/preflight/DDS callbacks executed in order.
Partial host success remains recorded if a later step fails; no rollback is
invented. Navigation modes associate localization metadata and log excerpts from
the frozen profile read-only; missing or unreadable provider metadata only warns.
Chassis modes do not inspect localization. No provider is started or signaled.

SIGINT, SIGTERM and SIGHUP request shutdown without interrupting cleanup. Remaining
owned handles are retried while the lease stays acquired and the manifest stays
unfinished; retry events are visible in the console. Only after groups drain are
the final manifest and summary written and ownership released. Manifest replacement
is atomic, so a failed final write preserves the prior readable record. Retention
only considers finalized older records; current, unfinished and malformed records
are kept. Tests delete only their own temporary history fixtures.

`startup_observer` captures an initial resource sample, then samples the supervisor
tree at 1 Hz and forwards child logs/key events to both the terminal and an
exclusive `logs/console.log`. Forwarding preserves bytes and handles partial writes,
partial event lines and a disconnected terminal. It does not follow log symlinks.
Sampling errors warn without discarding child logs; this observer has no ROS context.
Integration tests use a real example-robot bundle but bind launch/check/zero to
ordinary private Python children. Three signal tests prove cleanup and finalized
records after INT/TERM/HUP; none opens a serial port or sends ROS messages.
Separate domain-99 sandbox checks exercise real CLI/launch with the fictional
fake chassis, private lease and fake/private host operations. Their only velocity
publications are zero. `zero_command` accepts topic/discovery selectors, no speed
argument; it publishes five default-constructed Twist messages per topic and
reports missing receivers. Publication is a stop attempt, not proof of physical
stopping. Combined teleop tests also cover normal quit, parent interruption,
pidfd-targeted SIGKILL of only the owned keyboard, and publisher-conflict rejection.
They use private TTY/lease fixtures and fake chassis/host callbacks. Navigation
acceptance and production deployment remain outstanding.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --metas src/colcon.meta --packages-select nav_bringup
ROS_DOMAIN_ID=99 colcon test --packages-select nav_bringup --event-handlers console_direct+
colcon test-result --verbose
```
