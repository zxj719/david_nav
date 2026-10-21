# nav_bringup

Bringup, run recording and operations tooling for robot navigation stacks.
This package is robot independent: robot-specific values live in a
`<robot>_config` package.

## Status

| Component | State | Plan reference |
|---|---|---|
| `nav_bringup.lint.literals`: no robot frame names or robot topic prefixes in package code | available | plan §5.4 |
| `nav_bringup.navcfg.schema`: per-package `config/schema.yaml` format, catalog aggregation, value and profile checks, schema discovery | available (P2.1) | plan §12.1 |
| `nav_bringup.recorder`: run directory, manifest, `code.diff`, events, resource samples, summary, retention | available (PROGRESS P1.2a) | plan §13.2 |
| `navrun dump-params`: effective parameters of every running node into `params/effective/` (golden capture rules: helper nodes skipped, per-call timeout, retries, non-empty output; duplicate node names reported; output filename collisions fail without overwriting node dumps) | available (P1.2b) | plan §13.2 |
| `navrun monitor`: one node for behavior tree triggers, `/nav/events` recording, key topic rates and publisher counts, and map→robot TF lag (`metrics/topics.csv`, 0.2 Hz); rates, topics without publishers and TF lag percentiles in `summary.md` | available (P1.2b/P6.6 ingestion) | plan §10.3, §13.2 |
| Linked localization (`recorder.linked_process`): pid, start time, command line and config SHA-256 of the separately started localization in the manifest; what its logs gained during the managed run copied into `logs/loc/` (last 20 MB per file); read-only | available (P1.2c) | plan §13.2 |
| Sandbox integration checks (`sandbox_tests/`): `navrun monitor` and `navrun dump-params` against live processes in DDS domain 99; not collected by `colcon test`, run with `sandbox_tests/run_sandbox_checks.sh` | available (P1.2b) | RUNBOOK §5 |
| Escape and recovery triggers (`recorder.bt_triggers`, `recorder.bt_log_watcher`) from `/behavior_tree_log` → `events.jsonl`, terminal line, `logs/navrun.log`, per-node counts in `summary.md`; subscribed by `navrun monitor` | available (P1.4a) | plan §10.3 |
| `navrun doctor`: read-only checks (ROS/DDS environment, orphan Fast DDS segments, map yaml mode, duplicate robot_state_publisher, chassis serial device, ROS log directory size) | available (P1.5a, P1.5b) | plan §13.3 |
| `navrun doctor --live [--cmd-topic T] [--watch-topic T]`: stale ros2 CLI daemon (compared with `--no-daemon`; never started or stopped), several publishers on a command topic, publishers from more than one host (Fast DDS GID host bytes), map→odom transform missing (hint) | available (P1.5b) | plan §13.3 |
| `navrun up` | recorded navigation/chassis/teleop entry available; deployment acceptance pending (P4.2) | plan §13.1 |
| `navcfg render --profile FILE [--mode FILE] --output FILE` (`nav_bringup.navcfg.render`): a base parameter tree whose robot-specific values are `<bind:KEY>` markers, a binding table listing every parameter each profile key is written to (markers and table must agree), the robot profile and a mode overlay (profile values plus `nav2_overrides`, which may not touch bound parameters); PyYAML and the standard library only | available (P2.3) | plan §12.3 |
| `config/nav2_base.yaml` + `config/nav2_bindings.yaml`: Nav2 base parameters with frames, footprint, speed limits, goal tolerances, odometry and map topics bound to profile keys, which `config/schema.yaml` declares as required | available (P2.3) | plan §12.3 |
| `navcfg validate --profile FILE --mode FILE --site FILE --urdf FILE --bt-dir DIR [--source-root SRC]`: schema checks plus the frame, speed, precise-mode, trinary-map, teleop, diff-drive BT and POI-frame rules; warnings do not reject the configuration | available (P2.5) | plan §12.4 |
| `navcfg doc` | available; generates the workspace parameter catalog | plan §12.1 |
| `nav.launch.py` | available; consumes the prepared graph for the four supported modes | plan §7.1 |

Workspace [architecture](../docs/architecture.md) and [runbook](../docs/runbook.md)
distinguish implemented code from deployment and the remaining CLI work.
The read-only `ls/show` interfaces and frozen parameter/overlay exports are available.
`navrun check` revalidates a prepared run offline, with opt-in live observation.
`navrun status/down` contact the current cooperative `up` owner under its lease.
`navrun gc` previews or applies the shared completed-record retention policy.
`navcfg validate --robot NAME --mode MODE` selects source or installed resources
and validates them offline (P7.A2). `navrun selftest --mode tour|precise` lazily
loads the optional `nav_sim` package; it is not a reverse package dependency.
Select `--prepare-only` (write inputs without ROS), `--startup-only` (real private
domain-99 startup/cleanup without goals), or explicit `--allow-goals` (full experiment).
Only a closed-loop result with `complete=true` proves both POI and Dock completion.
The full runner is implemented; goal-bearing execution remains pending under the
current safety rules. See [nav_sim](../nav_sim/README.md) for guards and result semantics.

### Validate a robot configuration

```bash
navcfg validate --robot example_diffdrive --mode tour
navcfg validate --robot example_diffdrive --mode precise --site offline --source-root ./src
```

The named interface uses the shared up input discovery and offline bundle
preparation, including schema, profile/mode merging, Nav2/BT rendering, frame and
limit rules, URDF, map metadata/image, and selected POI file readability.
It checks the robot, mode and site identities against the selectors. It does not
call up host planning, resolve runtime executables, acquire a
navigation lease, create a run record, open hardware or start ROS.
Schemas use `--schema` if explicit, otherwise `--source-root`, otherwise the
installed resource environment. Package resources similarly use `--source-root`
or the installed package index; no robot-specific package code is imported.

Mode names: tour, precise, chassis and teleop; the selected robot must
supply the corresponding overlay. Navigation auto-selects a site only when exactly one is available;
otherwise pass `--site NAME`. Chassis/teleop need no site, map or URDF and reject
`--site`. Relative asset paths retain their package/site-relative meaning.
Warnings remain nonfatal. Exit 0 means valid offline configuration, 1 rejected
configuration, 2 invalid selectors/schema discovery or missing/inaccessible files.
Success does not certify executable availability, a live ROS graph or hardware;
read-only validation is not a transactional snapshot of concurrently edited files.

The original `--profile FILE --mode FILE --site FILE --urdf FILE --bt-dir DIR`
interface and its explicit `--base/--bindings` options are unchanged. Do not mix
these file overrides with `--robot`; use the explicit interface for custom files.

### Collect completed records

```bash
navrun gc --runs-root ~/nav_runs --dry-run
navrun gc --runs-root ~/nav_runs
```

The second command deletes replaceable artifacts: inspect the preview first.
Defaults retain the newest 30 complete runs within 20 GiB of logical file bytes,
and remove nonempty bags older than seven days (measured from run start).
`--max-runs`, `--max-total-bytes` and `--bag-max-age-days` override those limits;
zero is valid and may select all completed payloads. `manifest.yaml` and
`summary.md` are permanently retained, unchanged; `latest` is not rewritten.
Unfinished, malformed, missing-summary or unsafe records are conservatively
skipped, so the limits are not a guarantee on total disk usage. No ROS is started.

Dry-run acquires no lease and writes nothing. Application requires the navigation
lease (`--lock-path`, same path as the managed up) and a per-root `.gc.lock`;
concurrent collectors or active managed up fail closed. Manual processes do not
own that navigation lease: completion evidence, not PID discovery, protects their
unfinished records. Managed startup excludes its current record from the historical
budget.

Traversal/deletion is descriptor-relative on Linux, rejects symlink run roots,
and never follows payload symlinks. Selected root ancestors are trusted. This is
not a transaction or a defense against an administrator changing mounts or a
same-user writer racing the final unlink. Replacement detected during collection
aborts; earlier deletions may already have occurred and have no built-in recovery.
JSON reports planned targets and logical removed bytes, not physical space freed.
Exit codes: 0 completed/no-op/preview, 2 invalid/busy/failure, 130 interrupted;
on failure, `applied: false` does not prove that nothing was deleted.

### Inspect saved runs

```bash
navrun ls --runs-root ~/nav_runs
navrun show latest --params --diff --events --runs-root ~/nav_runs
```

These commands print JSON to stdout and never query ROS, create a run, acquire the
supervisor lock, or signal processes. `ls` lists newest first, leaving malformed
records visible as `unreadable`. `show` accepts an exact run ID or a `latest`
symlink whose target is exactly one relative run ID. Exit 2 means inspection
failed; the diagnostic omits parser source lines that could contain secrets.
`unfinished` means no recorded end time, **not** that a process is alive.

`--params` exports exact saved texts, grouped by `inputs`, `rendered`, and
`effective` (including the dump report when present). Empty effective output is
not replaced with rendered values. `--diff` compares the frozen robot profile to
that profile with its frozen mode overlay, then reports `nav2_overrides` and
explicit teleop selectors separately. This is **not** a rendered-to-effective
runtime comparison. Missing frozen inputs are reported as unavailable. Review
the effective dumps and commit chosen tuning changes to the original mode
overlay; do not turn this JSON export into a second snapshot configuration.
`--events` preserves complete JSONL lines verbatim and flags an incomplete tail;
it does not parse, validate or silently repair event payloads.

Exports may contain sensitive parameter/event/summary content: review before
sharing. Raw manifest commands, environment and linked process details are never
included. Reads are bounded to 8 MiB per file, 32 MiB per record, 4096 directory
entries and eight nested artifact directories. Oversized data is rejected, not
silently truncated. Mapping aliases are rejected. The selected root's ancestors
are trusted; the root and all record descendants reject symlinks (except the
validated `latest` selector), and files must be regular with one hard link.
Directory descriptors prevent path replacement from redirecting an open record.
Concurrent writes can still yield incomplete or mixed-generation evidence;
every report explicitly declares `snapshot_atomic: false`.

### Check a prepared run

```bash
navrun check --runs-root ~/nav_runs
navrun check --run-dir /absolute/prepared/run --live --strict --wait-timeout 10
```

Selection is explicit `--run-dir`, explicit `--runs-root` plus its direct `latest`
link, `$NAVRUN_RUN_DIR`, then the default runs root plus `latest`, in that order.
The two explicit selectors are mutually exclusive. Input directories must be
trusted supervisor-prepared runs, not arbitrary archives. No record or event log
is created. Offline validation calls the actual launch reader (required files and
lifecycle order), verifies frozen robot/mode/site identity, and re-runs schema and
cross-file safety rules on the saved profile, mode, Nav2 tree, map metadata, BTs
and URDF. Teleop includes its saved explicit selector overrides and hard-limit
checks. Source profiles are not reloaded; `checks.yaml` is not taken as proof.
Schemas come from `--schema FILE` (repeatable), `--source-root SRC`, or the current
installed schemas. This is not an atomic snapshot, whole-bundle hash verification,
external map-image/POI availability check, executable check or hardware acceptance.

Only `--live` calls the shared `navcfg` ROS observer and interface-contract check.
It never starts localization, sets an initial pose, sends a goal or velocity, or
changes parameters. Navigation modes support live checks; chassis/teleop return
2 rather than introducing a localization dependency. Offline safety errors always
return 1. Live findings retain the existing diagnostic policy: errors return 1
with `--strict`, otherwise they are printed with exit 0; warnings alone pass.
Invalid inputs/dependencies return 2; bounded pending localization returns 3;
interruption returns 128+signal after observer cleanup. `--wait-timeout 0` (default)
waits indefinitely for manual localization. `--sample-sec` and
`--parameter-timeout` must be positive finite values. The shared live output keeps
the `navcfg check` label; no second implementation of its ROS lifecycle exists.

### Inspect or stop the managed owner

```bash
navrun status
navrun down --timeout 2 --wait-timeout 30
```

These commands use the persistent `/tmp/navrun.lock` (or explicit `--lock-path`
for a private/test lease). They never send OS signals or select processes by
name/PID. Status reads the owned regular lock file without creating it and probes
its advisory lease without rewriting metadata. A held lock alone is insufficient:
the client requires a response from that incarnation's local cooperative endpoint,
verifies kernel peer PID/UID before sending an operation, and checks the instance
identity. Old PID-only locks, unsafe files, missing/unresponsive endpoints or
changed identities produce `unknown` with exit 2, never a kill fallback.

Each `navrun up` opens one Linux abstract UNIX sequence-packet endpoint with a
random per-run identity. No socket file, additional daemon or thread is created.
The existing recorder thread polls it at its existing 0.1-second cadence; at most
eight clients/requests are handled per pass, with bounded packets and a two-second
incomplete-client deadline. Requests are accepted only from the same user and
while the creating lease object/path is still owned. Record identity is published
before the observer starts. Slow or failed console/resource observation can make
the endpoint unresponsive; this is reported as uncertainty, not process absence.

`down` sets the supervisor's cooperative stop flag (existing SIGTERM-equivalent
exit 143), allowing the normal startup/child cleanup/terminal restoration/record
finalization sequence. Acknowledgment only means `stop_requested`. The client
reports `finished` and `cleanup_complete: true` only after a normal finalized
owner context and release of that same lease incarnation. A crashed owner,
replacement inode or intervening new owner cannot be mistaken for proved cleanup;
the client never retargets its request to the new owner. The abstract endpoint
closes before the lease is released; all accepted connections are closed too.

JSON exit statuses: 0 for verified status, finished cleanup, or an inactive managed
lease; 2 for unknown/invalid/operational failure; 3 for an acknowledged stop whose
cleanup remains pending at `--wait-timeout`; 130 if the client is interrupted.
`--timeout` must be positive and finite; `--wait-timeout` nonnegative and finite.
A response timeout does not prove that a stop request was never received.
There is no force-stop option. `active` means the supervisor owns its run, not
that Nav2 is ready. `inactive` does not prove that manual navigation, localization,
or arbitrary robot processes are stopped: these are not owned by
this `up` lease. Neither outcome is a physical braking/safety certification.

## Layout

```
nav_bringup/
├── nav_bringup/
│   ├── __init__.py
│   └── lint/
│       ├── literals.py         # no robot frame names / topic prefixes in package code
│       ├── node_attributes.py  # ROS Node subclasses must not shadow rclpy attributes
│       └── architecture.py     # task/executor layering and import direction
├── test/
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Code style

* `ament_flake8` with the ROS default configuration (99 columns, Google import order).
* `ament_pep257 --convention google` (summary on the first docstring line).

The `lint` checks above are kept because each guards a kind of problem that
actually happened: `literals` keeps frame names out of Python package code (the
same mistake in C++ — a costmap plugin hard-coding `base` — made DWB abort after
the base frame changed); `node_attributes` catches node members such as a method
named `handle` shadowing rclpy's own attribute; `architecture` keeps the layering
that lets task logic be tested without ROS. Docstring section and language gates
were removed in W46; documentation quality is a code-review matter.

## Testing

### Prepared launch entry (P4.1)

`nav.launch.py` consumes a pre-rendered run directory. It does not render, copy,
remove, probe or signal external processes, and never starts localization.
The selector arguments are `robot`, `site`, `mode`, `chassis:=on|off`,
`executor:=true|false`, `composition:=false` and required `run_dir`.
Robot/site/mode selectors must match the prepared bundle. Composition is available
as an explicit opt-in using the supported isolated-container components; default
graphs remain uncomposed and physical/performance acceptance is separate.

`launch.json` has version 1 and fields `schema_version`, `robot`, `site`, `options`.
Options are the fields of `launching.graph.GraphOptions`, except `run_dir` comes
from the launch argument; tuple-valued executor arguments are JSON arrays.
All selected parameters must already exist under `params/rendered/`:
`driver.yaml`, `adapter.yaml`, `nav2.yaml`, `lifecycle.yaml`, and the applicable
`map_server.yaml`, `robot_state.yaml`, `dock.yaml`,
`rviz.yaml` or `teleop.yaml`. Navigation also requires `localization_health.yaml`.
RViz uses `inputs/navigation.rviz`.
Lifecycle node order is checked against the graph before process construction.

The prepared tour/precise graph now uses `nav_executor/nav_task_manager`.
Precise mode has exactly one task owner: with WS enabled it serves both WS and
ROS Dock; with `--no-executor` it serves ROS Dock only. Tour without WS has no
task manager. The run-local `executor_profile.yaml` contains the **effective
mode-merged** profile, not merely the source robot YAML. Precise mode also passes
the complete `dock.yaml` through `--dock-config-file`. Both CLI input files are
checked before launch; historical bundles without them must be freshly prepared,
not launched using implicit David defaults. Other robot profiles must explicitly
provide `executor.grid_map_topic` and `executor.poi_store_file`; the offline
example now provides its own values.

The launch adapter eagerly resolves every executable before returning actions.
It allows only the configured number of driver respawns, shuts down on final
critical exits, and never respawns during shutdown. Each respawn uses a fresh
owned action instead of an unlimited background restart loop. External drivers
are omitted completely. The shared manager's WS-enabled CLI is protected by a
read-only action-server readiness gate and receives explicit argv, without a shell.
`phase:=all|core|executor` partitions the same frozen graph (default `all` for
standalone compatibility). `navrun up` starts `core`, completes its full checks,
then starts a second launch with `executor` only when the prepared bundle enables
WS. Both launches remain supervised, and executor shutdown precedes core shutdown.
The phase selector cannot enable an executor disabled in the prepared graph.

Consequently precise+WS Dock becomes available **after** Nav2 readiness/full
startup checks, together with WS, instead of as a separate early core process.
ROS-only precise Dock remains in the core phase. This timing change prevents two
task owners; Dock tuning and the Nav2 lifecycle order are unchanged. Preflight
refuses a recognized existing shared-manager executable and requests inspection,
without adding automatic kill patterns. Optional executable resolution still
checks installation before startup; no new package dependency cycle is added.
Do not combine independently launched managers with this graph.
Physical/no-goal ROS acceptance is pending.

This is not yet an accepted production replacement for aliases: shared-runtime
and physical acceptance remain pending. Bundle preparation, the convenience
keyboard launch, chassis adapters and recorded supervision are available.
No production alias has changed.
The sandbox launch checks use disposable Python children and an optional fake
Nav2 server; they never run the physical driver or submit a goal.

### Offline bundle preparation (P4.2)

`navrun up --mode-file /absolute/overlay.yaml` explicitly selects a complete mode
overlay instead of `<robot>_config/modes/<mode>.yaml`. The declared mode must
still match `--mode`; dry-run and startup freeze/check the same source bytes.
David's optional L2/L3 files are documented in `david_config/README.md`; no
experiment is selected by default. `nav2.velocity_smoother.enabled: false`
omits that process and lifecycle entry and connects controller/behavior output
directly to `chassis.adapter.input_topic`. Live checks follow the selected graph,
not the now-unused pre-smoother topic. No launch-time tuning override is added.

`navrun.preparation.prepare_bundle(GraphOptions, BundleInputs, catalog)` renders
all four modes without creating directories, importing ROS clients, discovering
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
Actual `up` without `--dry-run` enters the lock-scoped recorded workflow for
tour, precise, chassis and teleop. `--repo` supplies the repository recorded
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
Only then may the launch-managed WS executor start. If localization disappears
during observation, the default unlimited check closes its observers and re-enters
waiting, then performs a fresh complete check after recovery. Old findings are not
reused. Explicit positive `--wait-timeout` diagnostics instead return pending status
3; strict errors (1), operational errors (2) and signals do not retry. Neither
path restarts navigation or localization. After startup, continuous health is P6.7.
Chassis/teleop retain no localization prerequisite. Domain-99 integration checks run
the real tour/precise Nav2 graph with the example robot, fake driver and stationary
synthetic localization, without goals or a WS executor. They compare every declared
rendered value with complete effective dumps, require missing-key findings to match
those dumps, and verify strict rejection, provider independence and shutdown ownership.
These are startup/configuration checks, not driving or localization-quality acceptance.
Additional isolated tests cover localization loss/recovery and interruption;
pure/private-child tests cover deferred executor startup, failure and cleanup order.

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
executed by imports. Permission setup remains an
operator-owned sudoers task; the code never supplies a password or edits policy.

`navrun.startup_lifecycle.supervise` now executes an explicitly supplied startup
pipeline under an already acquired `RunLock`: localization wait → core launch → readiness
and live-check gates → optional executor launch → parameter dump → monitor/recorder → foreground
supervision → stop attempts and cleanup. Manual localization can wait indefinitely,
but launcher exits and requested signals remain observable throughout that wait.
Unexpected core/executor launcher or recorder exits fail the run, including exit code zero;
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
detached helpers must be separately owned. If cleanup cannot drain a group, the result
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

### Opt-in Nav2 composition

`navrun up --composition` freezes the selection in `launch.json`; omission keeps
the independent-process default. Direct `nav.launch.py` supports
`composition:=true|false`; an omitted selector preserves the frozen choice, and
older bundles without the field still select false. Chassis/teleop modes reject
composition rather than silently ignoring it. No default robot/mode file changes.

The seven Nav2 processes (map, controller, planner, behavior, BT navigator,
velocity smoother and lifecycle manager) share one critical
`component_container_isolated`. Optional map/smoother omissions still apply.
An explicitly selected C++ adapter joins that container; the default Python
adapter stays independent. Driver, robot-state publisher, scan bridge, dock,
RViz and deferred WS executor keep their existing ownership. Component failures
therefore have a larger process-level failure domain; the container is never
automatically respawned. Hardware A/B and the decision to enable it remain manual.
An already-running process explicitly named `nav2_container` causes `navrun up`
to refuse before host changes, even when the new request uses standalone mode.
Shared containers are not added to automatic kill patterns; other named component
containers, including localization providers, are not reclassified as navigation.

Logical node names, activation order, rendered parameters and topic remappings
are unchanged. Frozen parameter files are also passed to the container so nested
costmap nodes retain their YAML sections. Intra-process communication is not
implicitly enabled. Dry-run reports distinguish physical `processes` from
`components`; live checks/dumps still check every logical node. Only the exact
`/nav2_container` helper is marked skipped: Humble's ComponentManager disables its
own parameter services. All loaded components remain subject to full comparison.
Preflight verifies
all component resource registrations and readable library paths before any host
mutation; it does not load libraries or require unused standalone executables.

Unlike Humble's log-only loader failure behavior, `CheckedComponentLoader`
propagates rejected loads, service exceptions and a 90-second total load deadline
through the launch completion future. Launch then cleans up its owned stack.
Loading starts only after the owned container process starts; cancellation never
continues loading another component. Existing container-exit supervision and the
navrun stop/record/lease lifecycle remain active. Localization is never restarted.

## Shared navigation event recording (P6.6)

With a run directory, the existing monitor node subscribes to `/nav/events`
(`david_nav_msgs/msg/NavEvent`, reliable/volatile/keep-last depth 10). No second
node, spin thread, timer or motion client is added. A print-only monitor has no
unused event subscription. Subscription failure is reported and does not disable
the existing BT/rate/TF observations. Shutdown seals the event callback before
destroying its node; it does not wait for new DDS arrivals or replay history.
Finite timestamps beyond the system calendar range remain in JSONL unchanged;
the summary shows their numeric value and a calendar-range warning, not an exception.

Incoming records preserve the producer's Unix wall timestamp and all measurement
digits. The five JSONL top-level keys remain `stamp/source/type/severity/data`;
transport identity is stored at `data.__nav_event.event_id`. That reserved payload
key cannot be supplied by a producer. Existing locally generated events retain
their exact previous format, including six-decimal timestamp formatting. The
recorder and parent process cooperate using advisory file locks, flushing each
complete line before unlocking. This is not an fsync or disk-full transaction.

Input validation rejects empty/oversized identifiers, nonfinite timestamps or
JSON numbers, duplicate object keys, non-object data, invalid UTF-8, more than
32 nested levels and payloads over 65536 UTF-8 bytes. IDs are limited to 256
bytes; source and event type to 128 bytes. Warnings contain only failure types,
never untrusted event contents. An invalid event or failed write does not crash
the monitor, suppress later valid input or acknowledge any motion result.

The last 4096 successfully written event IDs form a bounded LRU duplicate window.
Duplicates refresh recency. Failed writes do not consume IDs, and close seals
late callbacks. This is **not** exactly-once or crash-durable deduplication:
evicted IDs and monitor restarts can admit duplicates, and partial disk writes
need offline inspection. Persisted IDs let offline consumers identify them.
The receiver never republishes, so this ingestion path cannot create an echo loop.

Task and Dock publishers are connected in nav_tasks. The same monitor node now
also publishes local `source=navrun` timeline records, including history written
before monitor startup. It keeps the recorded timestamp and uses a persistent
`navrun:<uuid>:<byte-offset>` identity. Incoming records bearing `__nav_event`
are never forwarded; the receiver suppresses its own persistent namespace even
outside its LRU window. No event payload, task result or motion policy is changed.

`RunEventStream` acquires a run-local `.nav_event_publisher.lock` lease and
atomically replaces `nav_event_cursor.json`. It never rewrites the timeline.
One pump reads at most 128 fragments / 1 MiB, with a 64 KiB record limit. The
monitor pumps at startup, on its existing sampling timer (default 5 seconds),
and once during cleanup. Partial trailing lines wait; oversized or invalid
complete records are skipped with category-only warnings. Detected replacement
or shrinkage seals the stream. A publication exception leaves that record for
the next pump; failed checkpoints are retried even while idle. Crash/restart
may replay IDs: a cursor is not a durable DDS-delivery acknowledgement, and
volatile subscribers may miss events. File locks are released before publication.

Outward data is a conservative projection: token-like `name/phase/stage/mode/
robot/site/exit_reason`, integer `pid/exit_code/return_code/signal/bytes/
removed_bytes`, boolean `found/exited_early/success`, plus `record_offset` and
`fields_omitted`. Raw commands, argv, environment, paths and exception text are
not forwarded. The complete local JSONL remains unchanged. This allowlist is
not a general secret detector for values intentionally placed in allowed fields.

Only one outward publisher may own a run. Publisher failure does not disable
the independent incoming recorder. This does not promise multi-monitor exactly-once
ingestion. Do not infer physical stopping, task success or production deployment
from these diagnostic records.

### Final parent events

Recorded `navrun up` now binds a short-lived final event publisher. After normal
cleanup has drained the original children and restored the terminal, the parent
collects linked logs and records the actual `run_stopped` result. Only then does
it start `python -m nav_bringup.recorder.event_flush`, keeping the navigation lease
until this helper is also drained and the manifest/summary are finalized.
The helper's exclusive run-local publisher lease is acquired **before** ROS init;
it cannot overlap a still-owning original monitor. It creates just one temporary
NavEvent publisher, no TF listener, subscriptions, timer, motion clients or daemon.
This is sequential teardown overhead, not another permanent DDS participant.

The helper resumes the monitor's cursor and persistent IDs, allows up to 0.5 s
for discovery, processes bounded batches toward its initial file-size snapshot,
and waits for reliable-writer acknowledgements within a total 3 s cooperative
budget. The parent imposes a separate 5 s execution deadline, then uses its
owned-process-group cleanup (1 s grace, escalation and retry if still owned).
Native SDK stalls cannot turn into a detached background process. Cleanup retries
can exceed the publication budget: ownership is not released on an unproven stop.
Failures remain diagnostics; the original navigation exit status is unchanged.
No-subscriber success, cursor progress and DDS acknowledgements still do not prove
application processing or disk persistence. Failed delivery does not roll back an
already advanced cursor or promise automatic durable replay.

The helper log is `logs/event_flush.log`. Its own final status is a local-only
`source=event_flush` event and appears in the summary if it warns; it is not fed
into another publisher. This explicitly ends the diagnostic lifecycle instead
of falsely reporting helper termination before it happens. Original local JSONL
records retain their format. Early configuration/host-preflight rejection keeps
only the local record and does **not** initialize ROS just to report the rejection.

The managed startup selects this tail after its monitor-stop step and true final
parent record. `StartupCommands` keeps the event-flush command explicit, and dry
runs start no helper. The final publisher independently refuses a live publisher
lease. Volatile early task events before monitor discovery can still
be missed: this is an observation channel, not an exactly-once audit transport.
Real ROS delivery/performance acceptance remains a separate safe-site check.
