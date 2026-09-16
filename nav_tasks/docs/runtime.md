# In-process task runtime

`nav_tasks.api.create_runtime(config, node, tf_buffer)` composes the ordinary POI
service through one public boundary. Importing `nav_tasks.api` still imports no
ROS, YAML or WebSocket implementation. Construction uses the real repository,
pose/grid adapters, goal resolver, POI skill, retry audit and task manager; only
the caller creates ROS nodes, TF listeners, executors and transport connections.
No action, observer or background task starts merely by constructing the runtime.

`RuntimeConfig` requires robot-specific paths, frames, topics, action name and
session explicitly. Numeric defaults retain the previous executor policies.
The optional named-poses source is read-only; the local POI store is writable.
The supplied `now` clock must use the pose timestamp time base. Profile/CLI
binding and parameter-schema ownership are not switched in this step.

The handle exposes `tasks: TaskApi`, `pois: PoiApi`, `poses: PoseSource` and
observer/shutdown methods. Ingress adapters must only import this public API.
Nonempty `dock_poi_ids` is currently rejected before file or ROS I/O: Dock must
not accidentally use ordinary goal snapping. This temporary gate remains until
Dock/Spin and their entry adapters share the same task/action ownership.

## Connection and shutdown ownership

- `start_loop_closure()` starts one supervised watcher; a repeated start reuses it.
  `stop_loop_closure()` cancels/joins it without canceling navigation. A reconnect
  can restart it. Observer crashes are logged and do not block final teardown.
- The outer root stops command ingress first and manages its event pump/outbox.
  It keeps the ROS executor spinning and the node alive while awaiting `close()`.
- Shutdown joins the observer, closes the task manager (including motion cleanup),
  closes the action backend, then removes grid and odometry subscriptions.
  It never stops an external node, executor or localization stack.
- Canceling a caller waiting for `close()` does not cancel the owned cleanup task.
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

Legacy executables, CLI/help, WS lifecycle, ROS spin ownership and Dock/Spin are
not switched here. Their final outer runtime must obey the lifetime contract
above; this factory alone is not complete runtime or hardware acceptance.
