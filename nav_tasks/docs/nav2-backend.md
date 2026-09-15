# Shared Nav2 backend (P6.2 integration substep)

`nav_tasks.nav2_client.Nav2Client` implements the one-attempt NavigationBackend
port. It uses one `ActionSession` for ownership; additional action types used by
Dock must share that session, never bypass its busy/cancel checks. The current
wrapper implements NavigateToPose only. Spin/Dock and actual runtime assembly
are still pending. No existing entry point is switched by installing this code.

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

`close` seals admission before cancellation. A backend-owned ROS client is
destroyed only after successful cleanup; injected clients remain their caller's
responsibility. A failed close must not be followed by unconditional client/node
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

Tests use fake nodes/messages and callback futures only. They exercise the actual
new TaskManager → NavigateToPoi → Nav2Client → ActionSession chain, including abort
and a subsequent task, but do not yet prove real ROS integration or G1–G16 wire
equivalence. Those remain mandatory before legacy replacement.
