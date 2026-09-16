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

## Before committing

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
