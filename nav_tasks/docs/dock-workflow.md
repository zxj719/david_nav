# Blocking Dock workflow

`skills.docking.workflow.DockWorkflow` extracts the synchronous two-stage sequence
from the existing node. It starts no ROS resources, worker or motion owner. A
future managed worker must execute it off the asyncio/ROS callback thread and
bridge DockMotionPort to the same Nav2Client used by ordinary navigation. The old
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
ROS action deadlines, worker joining, optional WS/same-manager ingress and real
Dock runtime acceptance remain mandatory before the legacy node can be replaced.
