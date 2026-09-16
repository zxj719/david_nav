nav_executor
============

0.1.0 (2026-09-16)
------------------

Current SPEC R1-R10 status (refactor workspace, not production deployment)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* R1: contextual malformed-command isolation keeps the WS connection alive;
  valid wire bytes remain frozen (43e110f, docs/adr-0007-message-isolation.md).
* R2: task crashes report on_error after owned cleanup; unproven cleanup never
  becomes IDLE (22f4087, nav_tasks/docs/adr-0008-crash-recovery.md).
* R3: synchronous, strongly owned queued feedback replaces orphan coroutines;
  late/canceled callbacks are fenced (06a4ce2, nav_tasks ADR-0009).
* R4: runtime logs the actual WebSockets version/path without global asyncio
  patches. Python metadata now selects pip websockets==16.1 for executor and
  simulation; ROS manifests no longer select the old apt key. No installation
  was performed, and full environment locking remains TODO-V2.
* R5: retained executable paths are forwarding shims and implementation lives
  in the symlink-installed Python package (884e4d5). Existing installation/help
  and dispatch checks do not mean production aliases have been switched.
* R6: passive profiling tool and eligibility checks are implemented (9bfa9d6),
  but actual 60-second idle CPU measurement and architecture choice remain
  pending; no CPU reduction is claimed (docs/adr-0011-r6-measurement.md).
* R7: legacy Dock text/regex consumers stay frozen; structured Dock/task events
  are additive (60a719c, nav_tasks/docs/events.md, docs/frozen_logs.md).
* R8: standalone Dock now defaults to /dock_goal; explicit overrides and
  mode-specific tolerances remain intact (b767ea9, docs/adr-0010-standalone-dock-topic.md).
* R9: required untracked production inputs were captured in the refactor
  baseline (93676a5); generated clutter is ignored (4388e37). Production's
  branch, index and worktree were not cleaned or rewritten.
* R10: system onboarding and configuration documentation were corrected
  (61b444d, f31b3d2). The retained waypoint launch message now says nav WS v1;
  dead-reckoning documentation names the actual waypoint default and explicit
  opt-in backend. The old semantic navigator is marked DEPRECATED with current
  public API guidance; historical parameters and executable behavior are retained.

Development history
~~~~~~~~~~~~~~~~~~~

The following entries describe stages as they landed. Statements such as
"pending" or "no runtime switch yet" below are historical, not current status.

* Add R6 passive Linux PID profiling with raw counters, monotonic intervals,
  weighted CPU, PID-reuse/error/interruption detection and explicit idle/window
  eligibility. No new dependency, target startup, ROS I/O or spin-loop change.
* Fix R8: standalone Dock robot data now defaults goal_topic to /dock_goal rather
  than Nav2 RViz /goal_pose. Preserve explicit overrides and the other fifteen
  policy values, including 0.05 rad/s settling; precise launch already used it.
* Integrate R2 task crash reports as existing on_error frames; verify successful
  cleanup and subsequent navigation through the public task adapter with fake I/O.
* Fix R1: normalize malformed commands with recoverable request context, isolate
  per-message failures on the same WS connection, and validate entire POI batches
  before mutation. Preserve valid wire bytes, unknown-action ignore behavior,
  cancellation propagation and reconnect on actual transport failure.
* Replace retained david_nav executor/Dock programs with forwarding shims and
  merge its compatibility precise launch into one shared owner. Keep standalone
  Dock defaults as installed robot data and preserve legacy algorithm tests with
  immutable references, separately testing real executable dispatch.
* Consume prepared navrun graph argv with one manager for ROS Dock and optional
  WS. Test effective robot/mode/site binding for David and the offline example;
  retain separate old david_nav entry points pending coordinated shim migration.
* Compose opt-in Dock and optional WS in one nav_task_manager process, applying
  actual ROS parameter overrides before runtime construction and rejecting recursive
  action endpoints. Preserve old executables, launch graphs and frozen help.
* Add owned off-loop multithreaded host quiescence, including SDK result-send tasks;
  retain resources on unproven shutdown and destroy only after two-phase settlement.
* Correlate shared-manager events by ingress source so ROS Dock completions cannot
  become unsolicited WebSocket arrival frames; preserve the old wire format and
  legacy empty-source behavior. Exercise full ingress/worker/runtime with fake ROS I/O.
* Load an explicit complete Dock parameter file from CLI/YAML/profile settings,
  preserving rendered mode tuning, endpoint precedence and separate capture/Dock
  frames. Bind the public task factory, retaining the outer Dock admission gate
  until its ingress and multithreaded ROS host are integrated.
* Migrate pure contracts with legacy import compatibility; no runtime switch yet.
* Add public TaskApi/PoiApi command adaptation and reconnect-persistent outbox.
* Test G1-G15 wire scenarios through the new manager, skill and Nav2 backend.
* Restore frozen status cadence/text and outbound audit helpers via public snapshots.
* Exercise the public task runtime factory with actual storage/skills/adapters and
  fake ROS I/O, including rollback failures and retryable shutdown.
* Extract proxy bypass, heartbeat, receive and reconnect components; join all
  connection children and reconnect promptly on sender failure without canceling
  motion or losing the shared outbox. Keep legacy entry points unchanged.
* Add immutable configuration, CLI/environment/YAML/profile precedence, headless
  section loading and frozen help. Keep robot literals in data and expose explicit
  task/transport bindings and per-value provenance; entry-point switching is pending.
* Add an ordinary-POI console entry, private ROS host and owned outer runtime.
  Graceful signals retain ROS callbacks until task cancellation settles; failed
  cleanup retains resources for retry. Dock/Spin and the legacy shim remain pending.
* Add opt-in, no-goal ROS/WS signal checks, separate from normal test collection.
