# Task and Dock observations (P6.6 / SPEC R7)

The public `nav_tasks.api.create_runtime` factory attaches publishers to its
caller-owned node. No extra node, ROS executor, timer, background queue or thread
is created. This is observation only: there are no event-driven motion commands.
No optional WebSocket connection is required for publication.

## Surfaces and identity

`/nav/events` uses `david_nav_msgs/msg/NavEvent`. Dock-enabled runtimes also
publish `std_msgs/msg/String` JSON on `~/events`, resolved against their actual
node name and namespace. Normal ROS topic remapping applies. Both publishers
use reliable, volatile, keep-last depth 10; late subscribers receive no history.

Each runtime creates a random producer identity and monotonic sequence, combined
as `event_id`. Task IDs may repeat and are not event IDs. Every message uses a
finite Unix wall-clock timestamp, independent of ROS/simulation time. JSON is an
object with finite numbers; measurements retain their original float values in
metres/radians/seconds. Failed encoding is diagnosed and dropped. This is not
durable delivery: publish exceptions are isolated, and shutdown does not wait for
DDS acknowledgements. No unbounded retry queue is added.

Task payloads include `task_id`, `sub_id`, `generation`, `action`,
`ingress_source`, `state`, `target_id`, `phase`, `remaining_distance`,
`ownership_released`, `detail` and `error_type`. They do not copy request bodies.
Dock payloads contain task/sub/generation correlation plus workflow measurements.
The private JSON adds `event`, `stamp` and `event_id`; global/private copies share
the same timestamp and identity. Either publication may fail independently.

## Meaning and compatibility

The task source is `tasks`; types include `task_admitted`, `task_phase`,
`task_progress`, `task_paused`, `task_cleanup_started`, `task_cleanup_failed`,
`task_fault`, and post-release `task_succeeded`, `task_failed`, `task_canceled`.
Failure/fault/cleanup-failure severity is `error`; other task events are `info`.
Only the post-release observations set `ownership_released=true`. This describes
software ownership, not measured physical stop or arrival accuracy.

External Nav2 cancellation still emits the frozen G12 paused event while the
manager owns the task. It is not a terminal observation. A crashed skill emits
`task_fault`; its final `task_failed` follows successful owned cleanup. Failed
cleanup retains ERROR ownership and has no premature terminal event. Internal
cancel, preempt and close are visible in this additive stream even where the old
WS contract intentionally suppresses a cancellation frame. Neither old TaskEvent
queues nor completion receipts change.

The Dock source is `dock`; global types use a `dock_` prefix. Private types are
`started`, `approach_checked`, `stage_started`, `measured`, `spin`, `after_spin`
and `no_pose`. Missing pose and blocked approach use `warn`; others use `info`.
An approach rejection emits its static-footprint result before any navigation
stage. Measurement events are not overall task-success events. The task manager
remains the authority for the eventual task outcome. Frozen `[dock]` log lines
are unchanged; golden tests run the observation hook across all legacy cases.

## Lifetime and limits

Task hooks execute synchronously on the owned asyncio loop; Dock hooks execute
on the existing joined worker, with attempt/stop fences. A shared publisher lock
serializes publication and destruction. The runtime registers cleanup before
opening publishers, so partially constructed handles survive failed rollback.
`settle` keeps publishers alive through motion cancellation and worker join;
`close` releases them after other runtime resources. Failed destruction retains
the handle for retry. Sealed owners suppress late emissions and cannot reopen.

Observer failures are isolated from skill results and legacy logs. Publication
is synchronous and can add DDS latency; this change does not claim zero CPU or
timing overhead, end-to-end delivery, or physical acceptance. The actual navrun
monitor now records these messages and publishes local lifecycle history with a
persistent cursor, while a managed temporary publisher handles the final tail.
The navigation health producer is also wired. Their deduplication and delivery
limits are documented in the workspace [event contract](../../docs/events.md).
No production alias is changed.
