# david_nav_msgs

Shared ROS interfaces. This package starts no node, owns no robot resources and
has no runtime parameters. The existing `MoveTo.action` is unchanged.

## NavEvent (P6.6)

`msg/NavEvent.msg` is the observation envelope intended for `/nav/events`.
It is **not** a motion, cancellation, admission or acknowledgement interface.
`RunTask.action` remains reserved and is not implemented in this refactor.

| Field | Contract |
|---|---|
| `event_id` | Nonempty producer-assigned identity, unique across producer lifetimes; unchanged on forwarding/replay. It is not the task ID. |
| `stamp` | Finite Unix wall-clock seconds at observation, matching existing run-event JSONL. Not ROS/simulation time; not a timeout clock. |
| `source` | Producing component, such as tasks, dock or navrun. Ingress metadata such as websocket/ros_dock stays in the payload. |
| `event_type` | Nonempty event name; maps to the existing JSONL key `type`. |
| `severity` | `info`, `warn` or `error`, matching the recorder vocabulary. |
| `data_json` | JSON object containing event-specific values. No NaN/Infinity, credentials or arbitrary command/environment dumps. |

The five semantic fields map directly to the existing recorder's
`stamp/source/type/severity/data` envelope. `event_id` adds transport correlation;
it does not require changing the frozen existing JSONL representation. Consumers
must not infer successful task ownership release or physical stopping merely
from an observational event. Task ID, sub-ID and admission generation belong in
task payloads so reused request IDs remain distinguishable.

The generated ROS class checks ROS field types, **not** the application constraints
above. Producers/consumers must validate nonempty identifiers, finite stamps,
allowed severities and object-shaped strict JSON. This interface alone does not
implement that validation, publishers, recording, QoS or replay/deduplication.
Those runtime integrations are the remaining P6.6 work, not silently optional.

Dock error values use metres/radians/seconds, preserving numeric precision rather
than parsing rounded text logs. SPEC R7 also requires the separate `~/events`
`std_msgs/String` JSON surface (`event`, `stamp` plus detail fields); NavEvent does
not replace it. Existing `[dock]` log lines must remain byte-for-byte unchanged
until dock_bench's separate future migration.

## Verification

The CMake-registered pytest suite checks the generated field schema, real CDR
serialization round trips (including Chinese text and Dock numeric values), and
the unchanged MoveTo goal/result/feedback contract. It does not initialize ROS,
create nodes, publish messages or send actions. Handwritten tests run flake8 and
Google-style pep257; generated code is not reformatted.

From the refactor workspace after sourcing Humble:

```bash
colcon build --symlink-install --metas src/colcon.meta --packages-select david_nav_msgs
ROS_DOMAIN_ID=99 colcon test --packages-select david_nav_msgs --event-handlers console_direct+
colcon test-result --test-result-base build/david_nav_msgs --verbose
```

Follow the workspace RUNBOOK's load limits and safety checks. These tests prove
the message ABI and packaging, not producer lifecycle or real ROS delivery.
