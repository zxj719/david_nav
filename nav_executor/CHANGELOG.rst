nav_executor
============

0.1.0 (2026-09-16)
------------------
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
