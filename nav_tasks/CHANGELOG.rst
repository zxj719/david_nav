nav_tasks
=========

0.1.0 (2026-09-16)
------------------
* Expose cancellation-safe runtime settlement separately from client/subscription
  destruction so the multithreaded host can join callback/result tails in between.
* Add shared-manager ROS Dock ingress, preserving topic/action names, busy logs,
  cancellation acknowledgement, complete feedback and retained cleanup ownership.
* Add optional post-release completion receipts and generation-matched cancellation,
  separately from unchanged legacy task events; preserve opaque ingress source metadata.
* Retain ingress ROS resources through close until the host proves executor quiescence.
* Bind complete legacy Dock parameters through a public immutable configuration
  API and a lazy ROS declaration adapter. Preserve all sixteen names and original
  launch/CLI overrides; require explicit robot/mode policy before construction.
* Migrate pure contracts with legacy import compatibility; no runtime switch yet.
* Add fake-testable task ownership/API, ports, registry, retry, supervision and geometry.
  Legacy handlers and runtime remain unchanged pending golden integration.
* Extract snap search and sequential POI/admin handlers behind pure ports; add
  per-attempt feedback and cancellation fences. Keep old runtime pending integration.
* Preserve zero remaining distance for external cancellation, distinct from abort.
* Add one-attempt Nav2 backend, shared action ownership and no-spin future helpers;
  retain timed-out cleanup, wait for terminal cancellation and stamp after readiness.
* Export PoiApi and MarkPoiResult for transport-independent administrative commands.
* Expose fenced target/phase observations and optional retry auditing without
  changing retry policy, motion ownership or legacy runtime entry points.
* Add ROS-free YAML POI repository and read-only named-poses importer with explicit
  frames, unchanged file format, byte comparisons and persistence fault injection.
* Add configured TF/odometry and grid subscriptions plus loop-closure observation;
  preserve legacy quaternion bits, sample semantics and owned-resource cleanup.
* Assemble ordinary POI services through a lazy public factory; retain cleanup
  ownership across canceled callers, deadlines and partial construction failures.
  Reject Dock configurations until integrated; do not switch legacy entry points.
* Pass explicitly configured loop-closure translation/yaw thresholds to the
  observer while retaining 0.05 m / 0.03 rad defaults.
* Add opt-in Spin to the same Nav2Client/ActionSession as NavigateToPose, with
  pure SpinGoal/SpinBackend contracts, signed angular feedback and explicit
  server allowance. Preserve one-attempt policy and block cross-action overlap.
* Retain multi-client construction rollback and failed teardown stages for retry;
  never destroy either client while shared motion cleanup is unresolved.
* Extract the blocking Dock workflow behind explicit worker ports, preserving
  approach admission, stage/finish policy and frozen log lines. Keep runtime
  integration disabled until managed thread/action bridging is complete.
* Move full-footprint static-map geometry unchanged into domain/footprint with
  legacy identity-preserving exports; verify workflow logs with dock_bench parsing.
* Add opt-in, independent action admission/result deadlines; retain late-handle
  cleanup and fail closed until terminal settlement. Leave runtime defaults and
  legacy Dock unchanged pending worker/backend integration.
* Add a managed blocking Dock thread and action bridge to the sole Nav2 backend;
  retain full feedback, frozen action logs and distinct deadlines. Join actual
  thread termination on cancellation, retain failed cleanup and fence stale success.
  Keep Dock runtime ingress/configuration and legacy entry switching pending.
* Assemble opt-in Dock through the public factory with explicit immutable policy,
  retained rotation-aware static maps and independent navigation-frame TF lookup.
  Route Dock POIs before snapping/retry, register direct pose tasks on the same
  manager and retain full feedback and owned resources across failed shutdown.
  Outer profile/CLI/ROS ingress and legacy entry switching remain pending.
