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

### Unit tests

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --metas src/colcon.meta --packages-select nav_bringup
ROS_DOMAIN_ID=99 colcon test --packages-select nav_bringup --event-handlers console_direct+
colcon test-result --verbose
```
