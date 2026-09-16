# ADR-0006: keep Dock's blocking worker under the single task owner

Status: accepted for the migrated components; runtime ingress wiring is pending.

PLAN §9 requires one TaskManager and one Nav2 client. SPEC P6 requires the Dock
algorithm and run_action facade to remain blocking work outside ROS callbacks,
not an asyncio rewrite of its two-stage/settling policy.

We keep DockWorkflow synchronous on an explicitly owned thread. Its blocking
motion port passes each request to the existing loop-confined Nav2Client and waits
on an internal future. No second client/session/executor is created. The eventual
Dock runtime must retain the specified MultiThreadedExecutor ROS callback model;
the current ordinary-POI outer host is not a completed Dock host.

Consequences:

- Pure policy retains its old timing, measured-pose verdict and frozen logs.
- Backend state and raw feedback callbacks remain on the TaskManager loop.
- Admission, result and cleanup deadlines are separate; a timeout never means
  motion ownership can be discarded. A cleanup error cannot become an advisory
  stage-two failure followed by an apparently successful measured verdict.
- Cancellation is cooperative. Pose/map callbacks must be bounded and thread-safe.
  A stuck callback keeps the owner busy rather than being detached or forcibly killed.
- Worker shutdown is joined, shielded and retryable; the shared backend lives until
  task/worker cleanup is complete. A successful old cleanup is not run again against
  a newer action. Stop fences prevent the old canceled-Spin/acceptable-pose corner
  from leaking a successful task result.
- Dock's default pose source, transient-local static map, configuration, ingress,
  structured events and same-process optional WS still need runtime assembly.
  Component and fake-action tests do not authorize production deployment or prove
  physical stopping.
