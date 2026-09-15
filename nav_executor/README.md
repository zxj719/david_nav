# nav_executor

Navigation WS v1 protocol and ingress adapters, separate from task ownership.
This first migration installs only pure commands and codecs; it does not replace
the existing runtime or add a runnable WebSocket entry point yet.

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
and preserve class/function identity. JSON serialization stays in the unchanged
legacy caller: key order, Chinese strings and three-decimal progress rounding
are preserved by G1–G16 and direct codec tests.

Malformed non-navigation frames retain their old KeyError/TypeError/ValueError
behavior in this relocation. R1 is a later separate red/green repair; do not
mistake this migration for a robustness fix. No handshake or new wire command
is introduced. Configuration schema is empty until the runtime starts consuming
profile keys in P6.4.

Only `nav_tasks.api` may be imported from the task package. The reverse dependency
is forbidden. Future WS ingress calls TaskApi rather than core implementation
classes. Pure protocol modules import no ROS, YAML or WebSockets, and importing
them starts no threads, processes or network connections.

The package's tests enforce these boundaries, documentation and formatting.
Existing deployment/CLI defaults are unchanged; production and alias 3 are not
modified. See [migration.md](docs/migration.md).
