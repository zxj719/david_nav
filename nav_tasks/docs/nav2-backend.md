# Shared Nav2 backend (P6.2 integration substep)

`nav_tasks.nav2_client.Nav2Client` implements the one-attempt NavigationBackend
port. It uses one `ActionSession` for ownership; additional action types used by
Dock must share that session, never bypass its busy/cancel checks. The wrapper
now implements NavigateToPose and opt-in Spin on the same instance. The ordinary
POI and Dock runtime, ingress and compatibility shims are assembled in the
refactor workspace; see [runtime](runtime.md) and [ADR-0006](adr-0006-dock-thread.md).
This is not a production deployment or physical-motion acceptance claim.

Construct on the task asyncio loop with an existing node, explicit action name
and server timeout. Real rclpy/nav2_msgs imports occur only when no fake client
and goal-message factory are injected. Importing the modules does not initialize
ROS, create a node, send a goal or start a thread. rclpy and nav2_msgs are declared
runtime dependencies; pure layers remain standard-library-only.

## Ownership and cancellation

One session permits one active attempt, including unresolved cleanup. It handles
server readiness, asynchronous admission and terminal result. A normal terminal
result, explicit rejection, or server unavailability releases the slot. A result
response must contain SUCCEEDED, CANCELED or ABORTED; an invalid/nonterminal status
or source exception does not prove that motion stopped and retains ownership.

On cancellation the session seals feedback, waits for the send future, cancels a
late accepted handle, and waits for both the cancel acknowledgement and a terminal
result. Cancel acknowledgement alone is not a stopped-goal guarantee. Concurrent
cancel callers join one strongly owned cleanup task. An already-terminal goal or
a rejection needs no additional cancellation request.
If the terminal result arrives before an already-requested cancel acknowledgement,
the execution path joins cleanup rather than releasing the slot early. Close and
new admission therefore cannot bypass that outstanding cancellation response.

The default cancellation *caller* deadline is 5 seconds. Timeout raises and keeps
the cleanup task and busy slot alive; it does not discard a late handle or pretend
that the backend is idle. Another cancel/close can join continued cleanup. Failed
cleanup can be retried. If submission raises without returning a trackable future,
the slot remains fail-closed: neither a goal ID nor a cancellation acknowledgement
can be established safely from that exception alone. Runtime integration must
surface that error for supervision rather than resuming navigation.

`close` seals admission before cancellation. Backend-owned ROS clients are
destroyed only after successful cleanup; injected clients remain their caller's
responsibility. Destruction attempts every owned client in reverse order and
retains failed stages for retry, without repeating completed stages. If Spin
client construction fails, the already-created navigation client is rolled back;
failed rollback raises RuntimeAssemblyError with a retained async cleanup handle.
Construction rollback does not close an externally supplied shared ActionSession.
A failed close must not be followed by unconditional client/node
destruction in the outer assembly root. No physical-stop guarantee follows
from fake action results or these software ownership tests.

## Compatibility and timing

R3 feedback ownership uses the event loop's strongly owned callback queue, not
unreferenced asyncio tasks. The mechanism was introduced with this backend and
is now protected by GC/thread/late-feedback tests and a no-bare-task-spawn check.
See [ADR-0009](adr-0009-feedback-ownership.md) for the old-source red/new-path green
comparison and the boundaries of this verification.

The wrapper retains legacy pose/quaternion conversion and detailed error text
for unavailable server, rejection and ABORTED, with no retry prefix. Retry and
feedback-report throttling belong to NavigateToPoi, not the backend. Latest raw
feedback still supplies the failure diagnostic distance. Each attempt builds a
fresh stamped goal **after** server readiness, not before a potentially long wait.

The 0.05-second readiness probe is active only during submission. It does not
change the old runtime's ROS spin loop or make an R6 CPU optimization claim.
All constructor intervals are explicit library arguments at this stage, not new
ROS parameters or consumed profile keys; runtime schema ownership follows wiring.

`ActionSession.execute` also accepts optional `send_timeout_sec` and
`result_timeout_sec`. Both default to None, so existing ordinary POI/Spin callers
are unchanged. The result budget starts after admission, independently of server
readiness and the send budget. Completed futures win even at a zero deadline,
matching the old blocking Dock wait helper. Pending timed waits use supervised
bridges; canceling a bridge never cancels the source ROS future.

These deadlines return SEND_TIMEOUT/RESULT_TIMEOUT only **after** cleanup has
confirmed rejection or terminal motion and any already-requested cancel ack.
Cleanup timeout/error instead raises and keeps the slot owned; a skill must not
interpret it as a settled segment failure and issue another goal. A TimeoutError
from the transport itself is not a local deadline and does not release ownership.
The Dock worker bridge selects the old 10-second admission and per-stage result
budgets, emits frozen action logs, and joins the blocking worker. An optional
synchronous on_timeout audit callback runs before deadline cleanup; even if it
raises, cleanup is still attempted. The API alone does not start an ingress;
the assembled Dock worker, runtime ingress and compatibility entry use it.

`navigate_pose` accepts a PoseRecord and forwards complete raw navigation feedback;
`spin_action` uses the same validated Spin factory without distance feedback. Both
return raw ActionOutcome to the Dock bridge for frozen action auditing, take
explicit stage deadlines, and use the same clients/session as navigate/spin.
They do not implement a second motion owner, snapping, retries or POI storage.

## Spin boundary

Supply `spin_action_name` to construct the second ROS client, or leave it None
to create no Spin resources. `spin_client` and `spin_goal_factory` may instead be
injected as a pair, as with navigation. There is still one ActionSession, not one
per action client. `cancel()` targets whichever action owns that slot; closing
either while rotation is unresolved does not destroy either client.

`spin(SpinGoal(target_yaw, time_allowance_sec), on_feedback)` implements the pure
SpinBackend port. Yaw is signed and is not wrapped or clamped. NaN/infinite and
non-float32-representable yaw, invalid duration, and booleans are rejected before
claiming motion. Duration uses the same int(seconds * 1e9) conversion as Humble
Duration, split into int32 seconds and nanoseconds. Feedback is angular distance
traveled in radians; it must not be forwarded as POI remaining distance in meters.

Readiness has an independent explicit `spin_server_timeout_sec` (default 5 s,
matching the old Dock wait). The goal's time_allowance is the Nav2 server-side
allowance; it is not a client-result or cleanup deadline. The current Dock workflow
retains separate send/result deadlines, settling, retry budget and frozen `[dock]`
audit lines; physical acceptance remains pending. This wrapper adds no retry or implicit settle check,
and its returned diagnostics are not substitutes for those frozen log lines.
Normal statuses 4/5/6 map to success/canceled/failed. No-action admission failures
release the slot; unresolved cleanup keeps both action types blocked.

Tests use fake nodes/messages and callback futures only. They exercise the actual
TaskManager → skill → Nav2Client → ActionSession chain, including preemption from
rotation to POI navigation, late handle/ack/result barriers, mixed injected/owned
clients, and failed construction/teardown. Preemption remains silent for the old
task; only explicit cancel emits paused. nav_executor separately tests G1–G16
components and an installed no-goal ROS/WS outer lifecycle. Dock and complete
legacy replacement acceptance are still required; no physical-stop proof follows.
