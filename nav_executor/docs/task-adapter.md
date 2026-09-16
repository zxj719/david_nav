# Task API ingress and reconnect ownership

This is an integration building block, not a deployed replacement entry point.
The old runtime, launch commands, ROS spin strategy and production code are unchanged.

`TaskCommandAdapter` depends on `nav_tasks.api` only. Navigation submits an
ALLOW-preemption task and returns after admission, not after arrival. Abort waits
for backend cleanup; its paused frame comes from the task event subscription,
not the immediate response list. Mark/rename use the separate `PoiApi` and never
take the motion slot. The existing parser and ordered wire builders remain intact.

The eventual runtime must:

1. Construct one adapter and one `ReliableOutbox` on the task event loop for the
   process lifetime, not for each WebSocket connection.
2. Own `adapter.pump_events(outbox.put)` in its task supervisor. Enqueue every
   immediate response and any heartbeat into that same outbox.
3. Supervise a connection-scoped `outbox.drain(websocket.send)`. On disconnect,
   cancel/join that drainer, retain the adapter/outbox and start a fresh drainer
   after reconnect. Do not cancel navigation merely because transport disconnects.
4. On final shutdown, safely close task/backend ownership while the event pump
   is still available; then cancel/join event consumers before `adapter.close()`.
   Adapter close only unsubscribes; it does not wake a pending reader or cancel
   motion. Backend close failure must not cause unconditional ROS client/node destruction.

No coroutine is spawned by these constructors. The event pump transfers each
dequeued event synchronously to `put`, with no intervening await. Enqueue freezes
the JSON bytes (Unicode preserved, default separators and field order). A failed
or canceled send retains its frame ahead of later frames. A lock prevents two
drainers from sending concurrently. The successful-send audit callback receives
a replay flag; audit-log wording remains the runtime's responsibility. A
post-send logging error does not requeue a successfully sent frame.

This is in-memory retention, not crash durability or exactly-once delivery. An
ambiguous send failure may replay a frame already received by the server. No new
handshake or server acknowledgement was introduced. Queues are unbounded as in
the old runtime; prolonged disconnection and slow subscribers can grow memory.

## Verification boundary

`test_task_wire_golden.py` uses the actual adapter, TaskManager, NavigateToPoi,
Nav2Client and ActionSession; only ROS messages/futures/client/pose/storage are
fakes. It checks ordered serialized G1–G15 frames, retries, two-leg arrival,
late-feedback fences, reused request IDs, administrative commands during motion,
and disconnect replay. `test_outbox.py` covers serialization, cancellation,
competing senders, audit failure and loop confinement. No ROS goal or velocity
is sent by these tests.

The new chain now supplies target/phase through immutable public TaskSnapshot
observations and covers exact G16 STATUS text across snap/anchor/idle transitions.
`app.status.print_status_loop` keeps the legacy 0.25-second poll, transition output
and active-only heartbeat without reaching into task-manager internals. The
runtime must supervise it and inject `app.logging.log_line` as its line sink.
An admitted generic task may have no target/phase until its skill describes it;
these labels render as `-`, rather than leaking wire payloads into the manager.

`nav_tasks.app.logging.RetryAudit` can be injected into NavigateToPoi to preserve
the G8 failure/retry/exhaustion lines. Retry notices are immutable, synchronous
observations before the delay or final outcome, not an alternative retry policy.
Success/cancellation does not invoke the observer; observer exceptions propagate
as they did with the old synchronous logger. `app.logging.OutboundAudit` plugs
into both ReliableOutbox callbacks to preserve pre-send errors and successful
terminal/replay lines (including G15). Healthy progress remains quiet.

G16 CLI/help migration is still pending. Actual WS lifecycle/ROS port assembly,
Dock/Spin, observer wiring at the assembly root and entry-point shims
remain pending; P6.1/P6.2/P6.4 must not be marked done based on these tests alone.

Legacy malformed administrative commands still raise KeyError/ValueError; R1
is deliberately left for its separate red/green repair. External CANCELED keeps
the frozen G12 NAVIGATING owner and emits paused distance 0.0. Invalid new POI
lookup preserves existing motion ownership, the intentional safety difference
already documented with the new manager. Detaching the adapter never cancels
an admitted task.
