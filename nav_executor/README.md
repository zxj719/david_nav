# nav_executor

Navigation WS v1 protocol and ingress adapters, separate from task ownership.
The package includes pure commands/codecs, a public-task-API command adapter,
and a reconnect-persistent outbox. The [WS lifecycle](docs/ws-client.md) now owns
connection-scoped receive/heartbeat/send work through an injected connector.
The [outer runtime](docs/outer-runtime.md) now provides an installed
`ros2 run nav_executor nav_task_manager` entry (also `navigation_executor`) for
ordinary POIs and explicit `--dock-config-file` Dock composition. `--no-websocket`
selects ROS-only ingress. The legacy launch/shim switch remains pending; production
is unchanged and separate old/new motion owners must not be launched together.

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

Malformed non-navigation frames retain their old KeyError/TypeError/ValueError
behavior in this relocation. R1 is a later separate red/green repair; do not
mistake this migration for a robustness fix. No handshake or new wire command
is introduced. The [configuration loader](docs/configuration.md) now resolves
explicit robot data, YAML, NAV_WS_URL and CLI values with per-key provenance.
Its schema declares consumed executor settings. The new entry owns its private
ROS context and preserves it until task cancellation and resource cleanup finish;
the old executable has not been switched.

Only `nav_tasks.api` may be imported from the task package. The reverse dependency
is forbidden. `TaskCommandAdapter` calls TaskApi and the administrative PoiApi,
not core implementation classes. Pure protocol modules import no ROS, YAML or WebSockets, and importing
them starts no threads, processes or network connections.

The package's tests enforce these boundaries, documentation and formatting.
Existing deployment/CLI defaults are unchanged; production and alias 3 are not
modified. See [migration.md](docs/migration.md) and
[task-adapter.md](docs/task-adapter.md) for runtime ownership contracts.
