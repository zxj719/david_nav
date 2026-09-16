nav_executor
============

0.1.0 (2026-09-16)
------------------
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
