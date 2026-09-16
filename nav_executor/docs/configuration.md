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

## Explicit Dock configuration bridge

`--dock-config-file PATH` (also `executor.dock_config_file` in profile/headless
YAML, or `dock_config_file` in flat executor YAML) reads a complete Dock mapping.
The file is either the sixteen flat legacy ROS parameters or the existing
navrun-generated `dock_navigator: {ros__parameters: ...}` tree. Multiple nodes,
wildcards, extra wrapper keys and incomplete/misspelled parameters are rejected;
this is not a general ROS parameter-file parser. `~` expands as with other paths;
relative paths retain current-directory meaning. Null/absence disables Dock data.

Use the already rendered precise-mode `params/rendered/dock.yaml` so the
profile's navigation frame, footprint, map and mode tolerances stay authoritative.
The loader does not infer precise tuning from the ordinary executor's profile,
does not conflate `frames.poi_capture` and `frames.nav_base`, and does not change
the old launch renderer. David's rendered 0.4363 rad / zero corrections and
0.05 rad/s stop threshold reach the task factory unchanged. Missing values never
fall back to the pure helper's 0.087 rad / two corrections.

`--dock-action-name` or an explicit executor YAML/profile `dock_action_name`
overrides the file's `action_name`; absent an override the file wins over the
legacy dataclass default. The effective Dock values appear in the startup audit;
the file path and endpoint override source remain in the provenance map.
`ResolvedConfig.dock_parameters` is immutable, ready for public
`nav_tasks.api.declare_dock_parameters(node, defaults)` to apply ROS overrides.
`ResolvedConfig.task_config()` binds `DockRuntimeConfig`; the old
`ExecutorConfig.task_config()` is the ordinary-only projection.

The new console now composes Dock when this complete file is supplied; IDs without
it still fail before ROS/storage. The host declares all sixteen parameters and
validates actual ROS overrides before task construction. Its startup audit contains
the final policy and endpoint (changed endpoint provenance is `ros:action_name`).
The loader alone leaves ROS arguments untouched. Backend and ingress names are
checked after expansion/remapping to reject recursive wiring. Legacy entry points
and launch graphs remain unchanged.

`executor.websocket_enabled` defaults to boolean true. YAML/profile false or
`--no-websocket` selects a ROS-only process; `--websocket` explicitly enables it.
CLI wins over YAML/profile, and string `"false"` is rejected instead of treated as
truthy. Disabled mode imports no websockets, changes no proxy environment, and
starts no WS heartbeat/connection/status/reprojection workers. Frozen legacy help
stays unchanged; these new options are documented here.

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
