# ADR-0009: synchronous queued feedback, no orphan coroutine

Status: P6.5 R3 verified. Mechanism introduced in 88d3bc4, reachable through the
shared runtime/retained entry migration completed in 884e4d5.

## Decision

ActionSession queues ROS-thread feedback with loop.call_soon_threadsafe. The loop
handle strongly owns the bound callback, attempt and message until dispatch;
the callback synchronously forwards to the skill/manager event queue. There is no
feedback coroutine, so no per-feedback asyncio task needs a separate reference.
This is the synchronous queue alternative explicitly allowed by SPEC P4 R3.
Actual asynchronous task work uses TaskSupervisor, whose existing GC test verifies
strong ownership through completion and exception retrieval.

Dispatch checks the current attempt identity, cancellation flag and settled flag.
Queued feedback is ignored if cancellation starts before dispatch. A completed
or canceled attempt cannot send progress to a replacement. Skill-level throttling,
latest-distance diagnostics, ordered WS frames and raw Dock feedback remain intact;
this change does not alter scheduling frequency, spin architecture or wire format.

## Red/green evidence

The same static no-unowned-spawn contract fails against the retained historical
david_nav/nav2_action_bridge.py at line 180 and passes for the new task runtime
(only core/supervisor.py may construct tasks). This is a read-only comparison,
not a claim that the current runtime still had the old defect or that a new
runtime algorithm was needed in this commit. The legacy source is not imported
by nav_tasks/nav_executor: architecture tests enforce that boundary; retained
executable shims dispatch to the new composition root. Historical bridge code
is still used by legacy golden/reference tests and awaits P7 isolation.

A real producer thread queues 100 fake feedback objects while the asyncio loop
is paused. After producer references disappear and GC runs, all messages remain
alive until ordered delivery on the loop thread, then become collectable. The
test rejects loop.create_task during delivery and verifies the task set did not
grow. Additional checks cover queued-before-cancel and stale callbacks after
cancel or normal completion followed by another attempt. No ROS context or action
goal is created; physical acceptance remains separate.

The message queue is not a bounded-load or latency guarantee. Unexpected callback
errors still use the loop exception policy; this audit does not add asynchronous
feedback processing or change error semantics. Production and manual imports of
the historical bridge have not been migrated by these tests.
