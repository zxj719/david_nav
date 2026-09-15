# nav_executor

Navigation WS v1 protocol and ingress adapters, separate from task ownership.
The package includes pure commands/codecs, a public-task-API command adapter,
and a reconnect-persistent outbox. It does not replace the existing runtime or
add a runnable WebSocket entry point yet.

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
the original G1–G16 still exercise the legacy runtime. G16 CLI/status and exact
runtime audit-log migration remain pending, so these are not full runtime acceptance.

Malformed non-navigation frames retain their old KeyError/TypeError/ValueError
behavior in this relocation. R1 is a later separate red/green repair; do not
mistake this migration for a robustness fix. No handshake or new wire command
is introduced. Configuration schema is empty until the runtime starts consuming
profile keys in P6.4.

Only `nav_tasks.api` may be imported from the task package. The reverse dependency
is forbidden. `TaskCommandAdapter` calls TaskApi and the administrative PoiApi,
not core implementation classes. Pure protocol modules import no ROS, YAML or WebSockets, and importing
them starts no threads, processes or network connections.

The package's tests enforce these boundaries, documentation and formatting.
Existing deployment/CLI defaults are unchanged; production and alias 3 are not
modified. See [migration.md](docs/migration.md) and
[task-adapter.md](docs/task-adapter.md) for runtime ownership contracts.
