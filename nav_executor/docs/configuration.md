# Executor configuration and compatibility

`app.config.load_configuration(argv, profile_path=...)` returns an immutable
ExecutorConfig, per-field source mapping and untouched ROS arguments. It opens
configuration files read-only and does not open the POI store, initialize ROS,
load a WebSocket library or start any runtime service.

Precedence (highest first): explicit CLI options, `NAV_WS_URL` for server_uri
only, `--config` YAML, selected robot profile, robot-independent dataclass defaults.
Paths expand `~` as the old CLI did; relative paths retain their working-directory
meaning. `--robot-profile` selects another robot data file without changing code.
Robot-specific frames/topics/store defaults are not literals in reusable modules.

Default profile bindings are `frames.poi_capture` (not frames.nav_base),
`frames.map`, `frames.odom`, `nav2.odom_topic`, `executor.grid_map_topic`,
`executor.poi_store_file`. Other recognized fields in the profile's executor
section override dataclass defaults; `executor.reconnect_delay_sec` maps to
`reconnect_delay`. Existing default odometry wiring is deliberately preserved,
not changed to the localization output. Site-specific values may still be passed
by the existing navrun binding. No automatic import of named poses is added.

`--config` accepts a flat executor mapping or the existing headless file's
`executor:` section. `load_headless_executor_section()` excludes the existing
launcher-only `nav_event_timeout`, `show_logs`, `record_bag` keys, without changing
the file format or old shell helper. Unknown runtime options are rejected.
Null disables optional source files. Numeric values must be finite/nonnegative;
heartbeat/poll/cancel deadlines and snap step must be strictly positive.

All old flags remain: `--server-uri`, `--poi-store-file`, `--pose-file`,
`--action-name`, `--reconnect-delay`, `--nav-retry-count`, `--nav-retry-delay`,
`--dock-action-name`, repeated `--dock-poi-id`. Dock IDs are trimmed/deduplicated
in input order. Other dataclass fields have matching hyphenated options, including
`--robot-base-frame`, `--odom-topic`, `--grid-map-topic`, `--global-costmap-topic`,
`--map-frame`, `--odom-frame`, `--server-timeout` and observer thresholds.
Argparse uses `parse_known_args` with abbreviation disabled; unknown ROS flags
are preserved. Incomplete/mistyped known flags now raise ConfigError explicitly,
rather than sometimes falling through to ROS. Invalid nonfinite/negative numeric
configuration is also rejected before any runtime resource is constructed.

`app.cli.build_help_text()` preserves the complete G16 help verbatim, including
its historical executable name. Newly exposed options are documented here and
in the dataclass/schema rather than changing the frozen text. The installed
`navigation_executor` entry handles `--help` before configuration loading, prints
`ResolvedConfig.log_line()` once, and forwards `ros_args` unchanged to ROS.
Without `--robot-profile`, it selects the installed david_config robot/david.yaml;
reusable runtime classes do not embed robot-specific frames/topics.

`ExecutorConfig.task_config()` maps explicitly to the public RuntimeConfig;
`ws_options()` maps transport timings. Loop-closure threshold overrides now reach
the supervised observer; status cadence reaches the outer root. Dock IDs remain
explicit and the outer root rejects them before ROS/storage I/O until integration.

Schema ownership remains unique: shared frames and Nav2 odometry declarations
stay with existing packages; reconnect_delay_sec stays with nav_bringup while
its existing launcher binding consumes it. New executor keys and the POI capture
frame are declared in this package's schema. The flat --config format is not a
replacement for the whole robot profile or its navcfg validation.

Tests compare help/defaults and supported legacy flags against the unchanged old
executor, verify precedence/provenance/immutable results, alternative robot
frames and topics, headless reads and invalid configuration rejection. Installed
CLI and graceful signals have separate no-goal sandbox checks; the old executable
shim remains pending. See [outer-runtime.md](outer-runtime.md) for ownership limits.
