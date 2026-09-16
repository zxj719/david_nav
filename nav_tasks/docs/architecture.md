# Task core contracts (P6.1)

The core is connected to the refactor CLI/retained shims through the nav_executor
composition root. Valid G1–G16 wire scenarios are covered by fake-only integration;
this is not physical acceptance and production entry points are unchanged.

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

`TaskContext.check_cancelled()` provides an explicit cancellation fence as well
as asyncio cancellation. Sequential skills check it before and after backend
attempts: even a backend that wrongly swallows CancelledError must not cause a
retry or an anchor segment after cancellation. Backends are still required to
propagate cancellation and clean up their actual goal.

## Events and compatibility

The additive [task/Dock ROS observation stream](events.md) is separate from the
legacy queues described below. Its hooks cannot acknowledge motion or replace
completion receipts, and the runtime owns publishers through settlement.

Events are immutable domain values, not serialized WS frames. A subscriber queue
is allocated only on explicit subscription. Consumers must drain and unsubscribe;
the core does not discard terminal events. Durable/reconnected delivery belongs
to the future reliable transport outbox, and feedback throttling belongs to the
navigation skill. A slow subscriber can accumulate events; no bounded-buffer or
durability claim is made by this in-process API.

Normal success/failure clears ownership. External `CANCELED` emits paused
progress with remaining distance **0.0** and retains the active task in `NAVIGATING`,
matching the legacy bridge and characterized G12
and review item C-2. Internal abort emits one paused event after cleanup; preempt
and close suppress the old task's cancellation/terminal events. R2 now lets skill
exceptions reach TaskSupervisor: its crash callback reports one FAILED event with
`执行器内部错误: <异常类型>` and owns a serialized backend cleanup operation. Error
delivery precedes cleanup completion; IDLE and the FAILED completion receipt only
follow successful cleanup. Failure retains ERROR ownership for explicit retry;
no busy retry loop or new motion is started. A later abort after a reported crash
does not append a misleading paused event. See [ADR-0008](adr-0008-crash-recovery.md)
for cancellation races and the distinction between the original crash and a new
cleanup error. R3 feedback ownership is covered by
[ADR-0009](adr-0009-feedback-ownership.md): queued synchronous callbacks retain
messages without creating unowned tasks; GC and stale-attempt checks guard it.

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

## POI navigation and administrative operations

`core/handlers/navigate_to.py` contains the sequential `NavigateToPoi` skill.
Its decoder resolves a target ID to an immutable POI snapshot before admission.
Its handler resolves the actual goal after admission, navigates to it, and only
after successful snap rechecks the anchor. An anchor that remains inaccessible
means snap success; an anchor that becomes admissible is a second segment, whose
arrival alone completes the task. A new segment reports STARTING, preserving the
existing status boundary without emitting an intermediate arrival. Each segment
receives a fresh retry budget.
Dock POIs must be routed to the Dock skill, not to this snap/retry path.

Feedback reporting preserves the previous per-attempt one-second throttle,
including its initial monotonic reference of zero. Each attempt has an additional
feedback fence, because retries and segments reuse the same task generation.
Callbacks from completed attempts cannot alter current progress.

`core/goal_resolution.py` extracts the existing ring search and a snapshot-fed
`GridGoalResolver`. It has no ROS subscriptions. Differential tests compare full
POI resolutions, candidate order and admissibility with the untouched legacy
implementation across 160 seeded map/configuration cases. Missing maps remain
permissive, missing robot pose disables distance sorting only, and only cost zero
is free. No origin-rotation support or new map-safety policy is claimed here.
Dock static-footprint admission is a separate check, not this point-snap policy.

`PoiCommands` handles marks and semantic-directory renames without a motion
backend or TaskManager reference. Capture errors and stale-pose text are retained;
the exact freshness boundary and future-timestamp behavior are unchanged. Storage
errors propagate, and actual atomic persistence remains the storage adapter's
responsibility. Ingress remains responsible for wire ACK/error/success encoding.

Compatibility boundary requiring integration review: unknown POIs decoded before
admission do not discard the old owner. The legacy unknown-POI branch cleared its
state without canceling the old backend; the new API does not reproduce that
unsafe ownership loss. This is not yet a change to a running legacy entry point.

## Extension and remaining integration

Adding a motion action requires a decoder, handler and `registry.register` call,
not another branch inside the manager. Wire actions remain owned by
`nav_executor`; it imports task contracts through `nav_tasks.api` only. A future
`dock_to` wire command would additionally need a message definition and protocol
documentation; this change does not add that command.

The shared single-attempt NavigateToPose backend and ROS-future ownership helper
are now implemented and tested with fake action clients; see
[nav2-backend.md](nav2-backend.md) and [ADR-0005](adr-0005-ros-futures.md).
Still pending: pose/storage/grid subscription adapters, Spin/Dock integration,
event-to-wire mapping and the replacement G1–G16 suite.
Only then can the old core be replaced. ROS task events, timeout/health integration
and the WS assembly root follow their separately tracked plan tasks.
