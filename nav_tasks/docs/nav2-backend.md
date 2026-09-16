# Shared Nav2 backend (P6.2 integration substep)

`nav_tasks.nav2_client.Nav2Client` implements the one-attempt NavigationBackend
port. It uses one `ActionSession` for ownership; additional action types used by
Dock must share that session, never bypass its busy/cancel checks. The wrapper
now implements NavigateToPose and opt-in Spin on the same instance. The ordinary
POI runtime is assembled; Dock orchestration/configuration remains pending.
No existing legacy entry point is switched by installing this code.

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
destruction in the eventual assembly root. No physical-stop guarantee follows
from fake action results or these software ownership tests.

## Compatibility and timing

The wrapper retains legacy pose/quaternion conversion and detailed error text
for unavailable server, rejection and ABORTED, with no retry prefix. Retry and
feedback-report throttling belong to NavigateToPoi, not the backend. Latest raw
feedback still supplies the failure diagnostic distance. Each attempt builds a
fresh stamped goal **after** server readiness, not before a potentially long wait.

The 0.05-second readiness probe is active only during submission. It does not
change the old runtime's ROS spin loop or make an R6 CPU optimization claim.
All constructor intervals are explicit library arguments at this stage, not new
ROS parameters or consumed profile keys; runtime schema ownership follows wiring.

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
allowance; it is not a client-result or cleanup deadline. Dock's future migration
must preserve its separate send/result deadlines, settling, retry budget and
frozen `[dock]` audit lines. This wrapper adds no retry or implicit settle check,
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
