# Blocking Dock workflow

`skills.docking.workflow.DockWorkflow` extracts the synchronous two-stage sequence
from the existing node. It starts no ROS resources, worker or motion owner. A
managed DockWorker executes it off the asyncio/ROS callback thread and
bridges DockMotionPort to the same Nav2Client used by ordinary navigation. The old
node/entry remain unchanged and the public runtime still refuses Dock configs.
This extraction is not a complete Dock integration or deployment.

DockConfig requires all policy explicitly: approach distance, DockTolerance,
SettleSpec, navigation/spin/settle timeouts. David uses its existing robot/mode
settings (0.5 m approach, 0.10 m/25 degrees acceptance, zero yaw retries); the
helper's historical 0.087 rad/two-retry defaults must not silently replace them.
The unchanged SettleSpec uses 0.05 rad/s over a continuous 0.4 s window, sampled
at 0.05 s. These software observations are not proof of physical stopping.

The flow computes the approach from the requested yaw, invokes the mandatory
full-footprint checker, then performs stages one and two. Stage-one failure
prevents stage two. An explicit cancel at either stage returns canceled. A
stage-two failure without explicit cancellation still proceeds to settled-pose
judgement, preserving the existing measured-position policy. Only this final
verdict can become task completion; the approach is not arrival at the Dock.

`skills.docking.approach.check_approach_footprint` rejects a missing map or a frame
mismatch, then calls the shared full-polygon static checker. That implementation
is relocated unchanged from david_nav.dock_approach; legacy exports refer to the
same classes/functions. It handles rotated map origins, polygon/cell area and
boundary contact, invalid geometry and out-of-map footprints. Unknown cells are
still allowed as before. Do not use ordinary POI snapping or a center-point test
instead, and do not move manually maintained POIs.

The finish routine remains blocking and unchanged. It waits for settling before
measurement and after each configured Spin, refuses translation after stage two,
and bases its verdict on the measured pose rather than Spin's boolean status.
The workflow passes Spin its server allowance and a distinct local result deadline
of spin_timeout + 5 s. Admission waits, action status logs and ownership cleanup
are the motion port's responsibility; its bridge must not return a false stage
result while silently abandoning unresolved motion.

The workflow reproduces old awkward outcomes too: an unsuccessful Spin followed
by an acceptable measured pose can succeed, even if the port marks that Spin
canceled. The eventual task owner must fence results during explicit cancellation
and must join the worker; this component alone does not enforce that cancellation
boundary. No new cancellation policy, retry budget, timeout meaning or frozen
`[dock]` line is hidden in this extraction. Worker crash, ingress rejection and
per-action admission/result lines still belong to later runtime adapters.

Tests extract only the three unchanged legacy workflow/logging methods from AST,
without importing its ROS node. Seventeen branches at three headings compare
result, action arguments, map-check pose, settling time and every log level/text.
The actual dock_bench parser must derive identical timelines/verdicts. Additional
tests enforce pre-action full-footprint rejection and legacy export identity.
The managed worker and per-action bridge are now covered by additional tests;
optional WS/same-manager ingress and real Dock runtime assembly/acceptance remain
mandatory before the legacy node can be replaced.

## Managed thread and action bridge

`adapters.ros.dock_worker.DockWorker` starts one explicit non-daemon thread per
admitted workflow. It owns a supervised join task and waits for actual thread
termination, not just a completion future. All workflow pose/map callbacks are
worker-safe injected ports. Production settling uses an interruptible Event.wait;
the existing finish/settle algorithms themselves are unchanged.

The blocking motion facade marshals each stage onto the owning asyncio loop.
`Nav2Client.navigate_pose` and `spin_action` share the same clients/ActionSession
as ordinary navigation. They add no POI lookup, snapping or retry. All ROS futures
are awaited by that one loop; the worker only blocks on its own cross-thread
answer future, never spins a ROS executor. Raw navigation feedback and the stage's
approach-distance offset reach a loop-only callback without dropping other fields.
The eventual ROS ingress must copy/forward those fields exactly as the old node.

Stage readiness/send budgets remain 5/10 seconds, with the workflow's independent
navigation or Spin-result budget. Deadline audit runs before owned cleanup; frozen
action log lines match old run_action for both navigation and Spin. An audit-hook
exception still runs cancellation cleanup. Cleanup exceptions are not converted
to a normal false stage verdict: the worker owner remains busy until explicit
cancel/close retry proves settlement.

Cancellation fences queued/new stages, requests cancellation of the active shared
action and joins both cleanup and the thread. Repeated execute cancellation or a
canceled close waiter cannot abandon cleanup. Blocking pose/map callbacks cannot
be forcibly stopped; ownership remains held until they return. A finished cleanup
is idempotent, so a late old waiter cannot cancel newer shared-backend work.
Exceptions retain ownership until cleanup succeeds; closing a worker does not
destroy the backend or cancel ordinary navigation when the worker is idle.

External server CANCELED/ABORTED remains an ordinary failed stage; the old measured
finish policy can still accept a correct final pose. Explicit task cancellation
is different: worker pose/sleep/admission guards and the final owner fence prevent
a successful task result even if the old finish algorithm could have accepted it.
TaskManager tests cover preemption to ordinary navigation only after real worker
exit, with no stale arrival. This is fake-action integration, not physical stopping
or a completed ROS Dock ingress deployment. See ADR-0006 for the thread decision.
