# In-process task runtime

The [Dock parameter bridge](dock-parameters.md) translates the existing complete
ROS parameter mapping into DockRuntimeConfig and applies launch/ROS overrides
without changing the declaration surface or creating motion resources.

`nav_tasks.api.create_runtime(config, node, tf_buffer)` composes ordinary POI and
explicitly configured Dock services through one public boundary. Importing `nav_tasks.api` imports no
ROS, YAML or WebSocket implementation. Construction uses the real repository,
pose/grid adapters, goal resolver, POI skill, retry audit and task manager; only
the caller creates ROS nodes, TF listeners, executors and transport connections.
No action, observer or background task starts merely by constructing the runtime.

`RuntimeConfig` requires robot-specific paths, frames, topics, action name and
session explicitly. Numeric defaults retain the previous executor policies.
The optional named-poses source is read-only; the local POI store is writable.
The supplied `now` clock must use the pose timestamp time base. Profile/CLI
binding belongs to nav_executor/navrun, not this factory. The refactor entry points
now supply those values; this does not switch production aliases or localization.

The handle exposes `tasks: TaskApi`, `pois: PoiApi`, `poses: PoseSource` and
observer/shutdown methods. Ingress adapters must only import this public API.
Nonempty `dock_poi_ids` without `RuntimeConfig.dock` is rejected before file or
ROS I/O: Dock must never fall back to ordinary snapping. Explicit DockRuntimeConfig
adds a retained map observer, worker and Spin on the same Nav2Client. The existing
nav_executor CLI/outer host now binds this policy from an explicit Dock parameter
file; its multithreaded host shares ROS/WS ingress. Retained legacy executable paths
are forwarding shims in the refactor workspace; production remains unchanged.

## Connection and shutdown ownership

- `start_loop_closure()` starts one supervised watcher; a repeated start reuses it.
  `stop_loop_closure()` cancels/joins it without canceling navigation. A reconnect
  can restart it. Observer crashes are logged and do not block final teardown.
- The outer root stops command ingress first and manages its event pump/outbox.
  It keeps ROS and the loop alive while awaiting `settle()`: this seals admission
  and joins motion/worker/session cleanup without destroying ROS resources.
  A multithreaded host can then quiesce callbacks before `close()` releases clients
  and subscriptions. A single-thread caller can still just await `close()` with ROS alive.
- Shutdown joins the observer, closes the task manager (including motion cleanup),
  joins/closes the optional Dock worker, closes the action backend, then removes
  Dock-map, ordinary-grid and odometry subscriptions.
  It never stops an external node, executor or localization stack.
- Canceling a caller waiting for `settle()` or `close()` does not cancel owned cleanup.
  On a timeout or cleanup failure, retain the runtime and retry `close()`; no
  later resource is destroyed ahead of failed motion cleanup. Successful teardown
  stages are idempotent. Do not release a surrounding run lock or destroy the
  node merely because a cancellation deadline expired.

Construction rolls back completed resources in reverse order, attempting all of
them. If a cleanup fails, `RuntimeAssemblyError` retains its retryable async
`cleanup` callable and chains the construction cause. The caller must retain that
exception and await cleanup successfully on the original loop before destroying
its node. Nested grid-construction failures retain partially created subscriptions
as well. A failed cleanup attempt does not discard callbacks; successful callbacks
are removed and are not invoked again. No task admission was exposed in this path.

Repository import occurs before ROS resource construction and retains the existing
incremental persistence semantics; resource rollback is not a file transaction.
See [site-data.md](site-data.md). No physical stopping guarantee follows from an
action result or these fake-I/O tests.

## Verification and remaining work

`nav_executor/test/test_runtime_assembly.py` exercises the public factory and
command adapter with actual task, storage and observation implementations. Fake
ROS modules supply only node/message/TF/action I/O; tests never initialize ROS.
Coverage includes startup/import logs, retries, observer restart/crash, canceled
close waiters, late terminal results, shutdown timeouts, construction rollback,
and repeated cleanup failures without double destruction.

The POI/Dock CLI/WS outer lifecycle is implemented in nav_executor, including
ROS pose/action ingress and same-process optional WS. Refactor executable/launch
dispatch and actual no-goal startup/cleanup checks have been exercised separately.
Complete POI/Dock task execution and physical acceptance are not proved by those
checks. See [the current Dock decision](adr-0006-dock-thread.md).

## Explicit Dock assembly

Pass a frozen `nav_tasks.api.DockRuntimeConfig` in `RuntimeConfig.dock`. All geometry
and finish policy are required: navigation rotation frame, global frame, static-map
topic, Spin endpoint, immutable footprint polygon, approach distance, xy/yaw
tolerances, correction count, navigation/Spin/settle deadlines and settle rate/window.
The inclusive occupancy threshold defaults to the old 50 and bounded pose lookup
to the old 2 seconds. The workflow retains its 0.05 s sampling period. These are
explicit library values, not new consumed ROS/profile parameters; the outer binding
uses robot/mode data. David precise mode uses 25 degrees/zero corrections; standalone
Dock retains 5 degrees/two corrections. Do not replace either with helper defaults.

Invalid configuration is rejected before repository reads or ROS resources.
Dock None leaves ordinary resource counts/policies unchanged. A Dock configuration
with no Dock POI IDs still registers the direct `dock` task action for ROS pose
and action ingress. It takes `{'pose': PoseRecord(...)}` and validates planar values
before preemption. The old configured-global-frame interpretation of incoming
coordinates is retained; ingress must not silently transform or reinterpret them.

`navigate_to` resolves the repository target once before admission. Configured Dock
IDs route to Dock before the ordinary resolver or retry policy runs. A whole Dock
failure is reported once with the old one-attempt ABORTED diagnostic/audit text;
it is never a license to rerun both displacement stages. Other POIs keep the old
snap and retry behavior. Only the complete settled Dock verdict produces arrival.

DockObservations uses reliable/transient-local/keep-last/depth-one static data and
copies a rotation-aware immutable OccupancyGridView. Missing/wrong-frame/invalid/
occupied or out-of-map footprints reject before any action. Its TF lookup uses
the separate navigation rotation frame, never the POI capture frame or a new odom
subscription. It logs the frozen TF failure line and returns None without touching
localization. Node/buffer/listener/executor remain externally owned and must stay
alive until worker cleanup succeeds.

Scalar task progress includes the approach-distance offset and old throttling.
An optional `on_dock_feedback(DockGoal, raw_wrapper, extra_distance)` callback gets
the complete unmodified feedback on the task loop for the ROS ingress; task ID,
sub-ID and generation identify the owner. The ROS adapter must preserve all message
fields when applying the distance offset and only forward to its matching active
goal. This API does not create an extra Dock action client or server.

`test_dock_runtime.py` drives the actual public factory, TaskCommandAdapter, real
worker thread, shared backend and real storage with fake ROS I/O. It checks map
gates, distinct TF frames/QoS, full feedback, direct-pose preemption, ordinary retry,
Dock no-retry, measured stage-two acceptance and retained shutdown resources.
