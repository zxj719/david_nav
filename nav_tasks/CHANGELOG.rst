nav_tasks
=========

0.1.0 (2026-09-16)
------------------
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
