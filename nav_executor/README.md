# nav_executor

The passive [R6 idle profiler](docs/idle-profiling.md) records an explicit existing
PID without ROS or process control. It checks window/identity/cadence completeness;
no robot CPU conclusion or runtime architecture change has been made.

Navigation WS v1 protocol and ingress adapters, separate from task ownership.
The package includes pure commands/codecs, a public-task-API command adapter,
and a reconnect-persistent outbox. The [WS lifecycle](docs/ws-client.md) now owns
connection-scoped receive/heartbeat/send work through an injected connector.
The [outer runtime](docs/outer-runtime.md) now provides an installed
`ros2 run nav_executor nav_task_manager` entry (also `navigation_executor`) for
ordinary POIs and explicit `--dock-config-file` Dock composition. `--no-websocket`
selects ROS-only ingress. The retained david_nav executable paths now delegate to
this root in the refactor workspace. Its precise launch uses one shared manager;
production is unchanged. Do not launch independent task owners together.

```python
from nav_executor.protocol.codec import parse_command, build_arrived_event

command = parse_command({'action': 'navigate_to', 'request_id': 'demo',
                         'target_id': 'poi', 'sub_id': 1})
assert build_arrived_event(command.request_id, command.sub_id) == {
    'event_type': 'on_arrived', 'request_id': 'demo', 'sub_id': 1}
```

`protocol/messages.py` owns the original command dataclasses and contextual
CommandParseError. `protocol/codec.py` retains parse_command and all ordered
event builders. Existing `david_nav.navigation_protocol` imports remain valid
and preserve class/function identity. The new outbox uses the same JSON settings
as the unchanged legacy caller. New-chain tests cover G1–G15 wire scenarios;
the original G1–G16 still exercise the legacy runtime. New status and audit helpers
cover G16 STATUS and exact G8/G15 logs through the new chain. CLI/help and
ordinary-runtime wiring are present, but this is not full Dock/runtime acceptance.

R1 now normalizes malformed fields into contextual MalformedCommandError and
isolates individual bad messages without reconnecting. Correlated failures use
the existing on_error frame; unknown actions remain log-and-ignore. Complete POI
directory validation precedes mutation. See [the R1 decision](docs/adr-0007-message-isolation.md)
for the malformed-batch policy and transport/cancellation boundaries. No handshake
or new wire command is introduced. The [configuration loader](docs/configuration.md) resolves
explicit robot data, YAML, NAV_WS_URL and CLI values with per-key provenance.
Its schema declares consumed executor settings. The new entry owns its private
ROS context and preserves it until task cancellation and resource cleanup finish;
the retained executable is now a thin forwarding shim.

Only `nav_tasks.api` may be imported from the task package. The reverse dependency
is forbidden. `TaskCommandAdapter` calls TaskApi and the administrative PoiApi,
not core implementation classes. Pure protocol modules import no ROS, YAML or WebSockets, and importing
them starts no threads, processes or network connections.

The package's tests enforce these boundaries, documentation and formatting.
Existing deployment/CLI defaults are unchanged; production and alias 3 are not
modified. See [migration.md](docs/migration.md) and
[task-adapter.md](docs/task-adapter.md) for runtime ownership contracts.

## 30-second inspection, reuse and extension

SPEC D3 selects **pip WebSockets 16.1** on Python 3.10/Humble, not the apt 9.x
distribution. `nav_executor` and `nav_sim` declare `websockets==16.1` in their
Python installation metadata; the ROS package manifests no longer request the
conflicting unversioned apt key. The retained `david_nav` shim depends on
`nav_executor`. Other versions are not claimed supported, and no global asyncio
compatibility monkeypatch is applied. This is one explicit dependency decision,
not a complete environment lock (TODO-V2 remains).

No software is installed by this refactor workflow. A future operator-managed
environment must provision that version before building/testing; `rosdep` alone
does not satisfy the pip requirement. Check the interpreter that will actually
run the entry point, e.g. `python3 -c "import websockets; print(websockets.__version__,
websockets.__file__)"`; the runtime also logs those values. Symlink/develop builds
can bypass dependency resolution, so successful colcon build alone is not proof
that another machine has the required distribution.

After loading the refactor environment, `ros2 run nav_executor navigation_executor --help`
prints the frozen CLI help without starting ROS or motion. A real launch additionally
requires the robot profile, site data and safe operator startup; this is not a command
to launch localization or send a test goal. See [configuration](docs/configuration.md),
the [generated parameter table](../docs/parameters.md) and
[outer runtime](docs/outer-runtime.md) for precedence, required Dock policy and ownership.

```text
WS codec/transport → nav_tasks.api → TaskManager → POI/Dock skills → Nav2 backend
                               ↘ task events → persistent outbox → WS
```

The [WS contract/AsyncAPI](../docs/protocol/nav_ws_v1.md),
[frozen log consumers](../docs/frozen_logs.md) and
[architecture/ADRs](../docs/architecture.md) describe compatibility boundaries.
For another Humble/Python 3.10 robot, supply a profile/site and the task/interface
dependencies; copying only nav_executor is not sufficient after the package split.
Keep body/navigation/POI-capture frames distinct. See [porting](../docs/porting.md).
Waypoint reuse selects `--action-name /waypoint_follower/navigate`; ROS-only ingress
uses `--no-websocket`. A non-WS client can implement/use `nav_tasks.api`; a backend
replacement implements the `NavigationBackend` port and is injected by the task
composition root. Future Foxy/Python 3.8 support is still TODO-V1.

Adding a hypothetical `dock_to` command would require a message/codec and AsyncAPI
definition, a task handler using existing skills, and registry wiring in the
composition root. Update the explicit wire adapter dispatch as required; do not
claim it is a universal dynamic command registry. Existing core state/transport
need not grow command-specific logic. **No dock_to command is introduced here**:
current Dock routing uses configured POI IDs in navigate_to.

## Required package gate

Build and test affected packages in the independent refactor workspace. Source
only Humble before building, then the refactor overlay for tests; load the formal
DDS profile and set the sandbox domain last. Do not install missing tools, start
localization, send goals, or publish nonzero velocity as part of these checks.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
MAKEFLAGS=-j2 nice -n 19 ionice -c3 colcon build --symlink-install \
  --metas src/colcon.meta --parallel-workers 2 --packages-select nav_executor
source install/setup.bash
set -o allexport
source src/david_nav/config/ros_env.conf
set +o allexport
export ROS_DOMAIN_ID=99
nice -n 19 ionice -c3 colcon test --packages-select nav_executor \
  --event-handlers console_direct+
colcon test-result --test-result-base build/nav_executor --verbose
```

`test/test_architecture.py` retains the direct import/literal gates and enforces
the shared transitive dependency policy. It also scans the seven new navigation
Python packages for Node subclasses against the **installed Humble Node source**,
without importing application nodes, initializing ROS or constructing a node.
The existing five concrete subclasses must be discovered, preventing a vacuous
pass. Properties/methods, class fields/slots and direct instance attribute writes
cannot hide inherited members, including private Node resource lists. Static
import aliases, reexports, relative imports, indirect inheritance and mixins are
covered; computed inheritance, runtime rebinding and arbitrary receiver aliases
are outside this AST analysis.

The only reviewed override allowance is
`nav_perception_iface.health_node.LocalizationMonitor.destroy_node`: it seals
callbacks and unregisters owned TF resources before delegating to the parent.
The allowance is exact, requires a same-method `super()` call and cannot exempt
an instance write; stale allowances fail. Structural delegation is not a proof
of execution on every path, so the existing health cleanup tests remain required.
Ordinary dunder lifecycle methods and Python-mangled double-private names are
not prohibited by this collision policy. The legacy Dock collision test remains
unchanged. Import-linter is still the separate uninstalled TODO-I1, not a claimed
passing tool invocation.
