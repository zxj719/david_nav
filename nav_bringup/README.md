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
| `navrun up` | planned (P4.2) | plan §13.1 |
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

### Prepared launch entry (P4.1; supervisor integration pending)

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

This is not yet a ready-to-deploy replacement for aliases: P4.2 must prepare the
complete bundle and finish navrun supervision; P4.3/P4.4 must supply the adapter
and teleop packages referenced by the graph. No production alias has changed.
The sandbox launch checks use disposable Python children and an optional fake
Nav2 server; they never run the physical driver or submit a goal.

### Offline bundle preparation (P4.2 component; up supervision pending)

`navrun.preparation.prepare_bundle(GraphOptions, BundleInputs, catalog)` renders
all five modes without creating directories, importing ROS clients, discovering
processes or opening devices. Its explicit paths work with source or installed
configuration packages. The caller still resolves chassis auto-detection and
legacy executor CLI arguments; the complete `navrun up` workflow is not yet exposed.

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
manifest integration remains in the supervisor work; `fingerprints.json` is its
prepared input, not a replacement manifest.

`write_bundle` adds these files exclusively to a newly recorder-owned directory.
It rejects existing artifacts and symlinked destinations before writing, and
writes `launch.json` last. A failed write leaves diagnostic partial files without
publishing a launchable bundle; it does not overwrite or delete existing records.

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

`navrun.process_backend` provides the Linux adapters for future up integration:
read-only psutil snapshots reject incomplete views, and signaling opens a pidfd
before rechecking PID/creation time/user/argv. It has no PID-only fallback, shell,
sudo or permission change. These adapters are not called by imports or existing
entry points. Tests replace all psutil/pidfd signal operations with fake objects;
no production residual has been stopped or restarted during this implementation.

`navrun.locking.RunLock` implements the shared `/tmp/navrun.lock` lease for the
future `up` supervisor. Construction does no I/O. Acquire is nonblocking, checks
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

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --metas src/colcon.meta --packages-select nav_bringup
ROS_DOMAIN_ID=99 colcon test --packages-select nav_bringup --event-handlers console_direct+
colcon test-result --verbose
```
