# Task core contracts (P6.1)

The core is executable with fake skills, but **is not wired into any production
or refactor launch entry point yet**. The legacy core remains the runtime owner
until the replacement passes the complete G1–G16 integration suite.

## Ownership and concurrency

`TaskManager` implements the public `TaskApi`: `submit`, `cancel`, `status`,
`subscribe_events`, `unsubscribe_events` and `close`. It owns one motion skill;
POI marking and directory updates are administrative commands, not motion tasks.
Registrations contain a pure synchronous decoder and an async handler. Decode
to a detached immutable value before admission; an invalid request must not
cancel a valid owner. The handler receives generation-fenced feedback and returns
a typed `NavOutcome` rather than pushing its terminal state into the manager.

All state is confined to one asyncio loop. Calls from another loop are rejected.
There are no thread locks. A loop-local `asyncio.Lock` serializes *admission and
cleanup awaits*, not navigation: single-thread execution alone does not prevent
two coroutines from interleaving across an await. `submit` returns after admission;
the receive loop never awaits navigation completion. Every background coroutine
created by the core is owned by `TaskSupervisor`, which retains strong references
and retrieves exceptions. Never use `spin_until_future_complete` in adapters.

Preemption policy is explicit per request: WS will pass `ALLOW`, while rviz will
pass `REJECT`. The API default is `REJECT`. Replacement motion cannot start until
both backend cancellation and the old skill's `finally` cleanup have completed.
The backend cancel contract includes any pending goal-handle admission window.
Neither a timeout nor an exception is evidence that the backend has stopped.
On cleanup failure, the old task remains owned in `ERROR`; retry cancel/close.

An accepted API transition is an owned operation. Canceling its *waiter* (such as
disconnecting an ingress) does not cancel that operation or accepted motion.
Use `cancel(task_id)` explicitly. `close` immediately seals admission, including
requests already queued behind a cancellation barrier, and joins cleanup. A
canceled close waiter does not abandon cleanup; repeated close calls join it.
Failed close can be retried but never reopens admission.

Task IDs may be reused. Each admission gets a new internal generation and a new
active object. Feedback is accepted only for that active object, before terminal
completion and before internal cancellation begins. Old feedback cannot modify a
new task even if both external IDs are identical.

## Events and compatibility

Events are immutable domain values, not serialized WS frames. A subscriber queue
is allocated only on explicit subscription. Consumers must drain and unsubscribe;
the core does not discard terminal events. Durable/reconnected delivery belongs
to the future reliable transport outbox, and feedback throttling belongs to the
navigation skill. A slow subscriber can accumulate events; no bounded-buffer or
durability claim is made by this in-process API.

Normal success/failure clears ownership. External `CANCELED` emits paused
progress and retains the active task in `NAVIGATING`, matching characterized G12
and review item C-2. Internal abort emits one paused event after cleanup; preempt
and close suppress the old task's cancellation/terminal events. A programming
exception reports failure and retains `ERROR` ownership until explicit cleanup;
automatic crash cleanup and its dedicated R2 regression belong to P6.5. This
substep does not claim that R2/R3 are fixed in the legacy runtime.

## Backend, retry and geometry

`core/ports.py` defines the navigation, pose, POI repository, goal-resolution and
clock contracts. `NavigationBackend.navigate` executes **one** attempt. The core
`RetryPolicy` retries only FAILED (default one retry after one second), propagates
asyncio cancellation and unexpected exceptions, and preserves the exact exhausted
failure text `导航失败(共尝试{n}次): {detail}`. An injectable monotonic clock makes
retry-window cancellation deterministic in tests. Pose freshness must use the
pose source's time base, not this interval clock.

`domain/geometry.py` supplies quaternion/yaw and planar transform helpers. Finite
nonzero quaternions are normalized scale-safely, and wrapped angles use [-pi, pi).
`transform_pose` requires an explicit `target_frame` because `Transform2D` itself
does not carry frame names. No robot-specific default is inferred. Existing
legacy geometry callers are not switched by this addition.

## Extension and remaining integration

Adding a motion action requires a decoder, handler and `registry.register` call,
not another branch inside the manager. Wire actions remain owned by
`nav_executor`; it imports task contracts through `nav_tasks.api` only. A future
`dock_to` wire command would additionally need a message definition and protocol
documentation; this change does not add that command.

Still pending: NavigateToPoi/snap and administrative handlers, adapters for the
ports, shared Nav2 client, event-to-wire mapping and the replacement G1–G16 suite.
Only then can the old core be replaced. ROS task events, timeout/health integration
and the WS assembly root follow their separately tracked plan tasks.
