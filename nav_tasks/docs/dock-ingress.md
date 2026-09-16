# Dock ROS ingress and ownership receipts

`api.create_dock_ingress(tasks, DockNodeConfig(...))` constructs an empty,
loop-confined owner. `open(node)` lazily creates the old depth-one PoseStamped
subscription and NavigateToPose action server in one ReentrantCallbackGroup.
The host owns the node, TF listener and **MultiThreadedExecutor with at least two
callback threads**. No new Nav2 client, ROS context, executor or motion worker is
created by ingress. Pass `ingress.on_feedback` through the task factory's
`on_dock_feedback` callback (a late-bound closure can resolve construction order).

The topic callback detaches x/y/yaw and returns without waiting for navigation.
As before, the incoming header is not transformed and z is ignored: configured
Dock global_frame determines the planar target. The action execute callback
waits on an internal concurrent future from its ROS callback thread; it never
spins ROS and cannot run on the manager loop. The actual synchronous docking
policy still runs on the existing managed DockWorker, with one shared Nav2Client.

ROS requests use REJECT admission. Goal-request callbacks reject a busy manager;
if two requests were accepted before execution, the manager admits one and the
other action aborts with the frozen legacy warning. Busy topic targets are
ignored with their old warning. There is no separate competing motion lock.
Other ingress can preempt according to its existing ALLOW policy. Stop seals
admission; a partial open never admits even if its completed subscription fires.

## Completion is not a legacy event

`TaskApi.submit(request, on_settled=callback)` optionally sends one immutable
TaskCompletion receipt **after ownership is released**, including preemption and
close. An admission rejection has no receipt. Normal success/failure retains its
outcome; completed cancellation cleanup returns CANCELED unless an earlier R2
crash has already established a FAILED outcome. An exception event or
cleanup timeout is not proof of release, and no receipt is issued yet. Losing a
submit/cancel waiter does not discard the manager's callback. A broken observer
is logged without corrupting the already-released state. R2 now automatically
requests cleanup for skill crashes; ingress cleanup retries share the manager's
transition lock and cannot duplicate release or cancel a newer generation.
The legacy G12 external-cancellation state remains unchanged.

Ingress subscribes to task events only while a request is active, so it can
request cleanup after a retained error/paused outcome. It retries failed cleanup
while retaining its action callback, without interpreting timeout as a motion
result. `cancel(id, generation=...)` checks both identity and admission generation
inside the manager transition lock: late cleanup cannot cancel a replacement
that reuses an ID. Receipts notify completion without a new periodic idle poll.
The only short poll is the ROS cancel acknowledgement gap: rclpy marks CANCELING
after the cancel callback returns. Early cancellation admits no motion and waits
through that gap before the execute callback marks CANCELED.

Raw feedback copies current_pose, navigation_time, estimated_time_remaining and
number_of_recoveries, adding the later-stage offset only to distance_remaining.
Task ID, sub-ID, generation, ownership state and cancellation fence publication.
A feedback-publication error requests owned cancellation and aborts only after
cleanup. Ordinary WS scalar feedback and the worker policy are unchanged.

TaskRequest/TaskEvent now carry opaque `source` metadata (empty by default).
Core preserves it without interpreting it. ROS ingress sets `ros_dock`; WS sets
`websocket`. The WS adapter forwards only its own or legacy-empty-source events,
preventing unsolicited ROS-task arrivals from becoming server-request results.
The wire format is unchanged; general event subscribers still observe all tasks.

## Required host shutdown order

1. Call ingress.stop() on the manager loop; stop WS admission if enabled.
2. Await ingress.close() and task-runtime.settle() **with ROS and the loop alive**.
3. Stop/join the spin driver, then await successful executor shutdown/quiescence.
4. Call task-runtime.close() and ingress.destroy(); retry failed destruction stages.
5. Destroy the external listener/node/context.

Do not run a blocking executor shutdown on the manager loop: a ROS callback may
still need to marshal its last operation onto that loop. Quiescence includes
rclpy's action-result/send tasks queued after our execute callback returns. `close()`
therefore deliberately does **not** destroy the action server/subscription, and
`destroy()` cannot itself prove the host's executor invariant. A host losing its
context or a permanently stuck callback requires explicit recovery, not false
completion or early node destruction. None of these receipts proves physical
braking/settling beyond the existing motion and observation contracts.

The nav_executor outer console now wires this multithreaded sequence and optional
WS in one process; a full explicit Dock parameter file is required. Legacy
node/entry points remain unchanged. Current tests use actual
callback threads, manager, DockWorker, runtime and protocol adapter with fake ROS
constructors/clients/futures only; no ROS graph, action goals or velocity commands
are published. Real host/no-goal lifetime verification and manual motion acceptance
are separate gates.
