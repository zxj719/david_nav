"""Immutable executor values and explicit CLI/environment/YAML/profile provenance."""

from dataclasses import asdict, dataclass, fields
import json
import math
import os
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Sequence

from nav_tasks.api import ConfigError, RuntimeConfig

from nav_executor.adapters.transport.ws_client import WsOptions


@dataclass(frozen=True)
class ExecutorConfig:
    """Explicit robot values plus legacy numerical and protocol defaults.

    Robot-specific defaults come from data, never literals in reusable code.
    Paths are normalized to expanded strings for compatibility with the old CLI.
    """

    poi_store_file: str
    robot_base_frame: str
    odom_topic: str
    grid_map_topic: str
    map_frame: str
    odom_frame: str
    server_uri: str = 'ws://127.0.0.1:8101'
    pose_file: str | None = None
    source_poi_file: str | None = None
    heartbeat_interval: float = 30.0
    loop_closure_poll_sec: float = 1.0
    action_name: str = '/navigate_to_pose'
    server_timeout: float = 10.0
    slam_session_id: str = 'live_session'
    global_costmap_topic: str = '/global_costmap/costmap'
    nav_goal_snap_radius: float = 0.5
    nav_goal_fallback_snap_radius: float = 1.0
    nav_goal_clearance_radius: float = 0.1
    nav_goal_max_snap_radius: float = 3.0
    nav_goal_snap_step: float = 0.25
    reconnect_delay: float = 3.0
    nav_retry_count: int = 1
    nav_retry_delay_sec: float = 1.0
    dock_action_name: str = '/dock_to_pose'
    dock_poi_ids: tuple[str, ...] = ()
    cancel_timeout_sec: float = 5.0
    loop_closure_translation_m: float = 0.05
    loop_closure_yaw_rad: float = 0.03
    progress_min_interval_sec: float = 1.0
    pose_stale_after_sec: float = 0.5
    status_print_interval_sec: float = 1.0 / 0.3

    def legacy_values(self) -> dict:
        """Project the original config mapping for compatibility checks.

        Returns:
            Original keys and types, without newly exposed options.
        """
        result = asdict(self)
        for name in (
            'map_frame', 'odom_frame', 'source_poi_file', 'cancel_timeout_sec',
            'loop_closure_translation_m', 'loop_closure_yaw_rad', 'progress_min_interval_sec',
            'pose_stale_after_sec', 'status_print_interval_sec',
        ):
            del result[name]
        result['dock_poi_ids'] = list(self.dock_poi_ids)
        return result

    def task_config(self) -> RuntimeConfig:
        """Translate configuration into the public task factory contract.

        Returns:
            Immutable task values; Dock IDs remain explicit for its admission gate.
        """
        values = asdict(self)
        values['server_timeout_sec'] = values.pop('server_timeout')
        for name in ('poi_store_file', 'pose_file', 'source_poi_file'):
            values[name] = Path(values[name]) if values[name] is not None else None
        return RuntimeConfig(**{field.name: values[field.name] for field in fields(RuntimeConfig)})

    def ws_options(self) -> WsOptions:
        """Extract transport timings without importing a WebSocket library.

        Returns:
            Explicit URI and connection timing values.
        """
        return WsOptions(self.server_uri, self.heartbeat_interval, self.reconnect_delay)


@dataclass(frozen=True)
class ResolvedConfig:
    """Effective configuration, per-field provenance and untouched ROS arguments."""

    config: ExecutorConfig
    sources: Mapping[str, str]
    ros_args: tuple[str, ...]

    def log_line(self) -> str:
        """Serialize effective values and their sources for startup auditing.

        Returns:
            One JSON-bearing log line without the legacy logger prefix.
        """
        return 'effective config: ' + json.dumps(
            {'values': asdict(self.config), 'sources': dict(self.sources)}, ensure_ascii=False)


def _read_mapping(path):
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f'{path}: {error}') from error
    if not isinstance(data, dict):
        raise ConfigError(f'{path}: configuration must be a mapping')
    return data


def _executor_section(data, path):
    section = data.get('executor')
    if not isinstance(section, dict):
        raise ConfigError(f'{path}: executor section must be a mapping')
    # These are launcher/recording settings, not executor runtime options.
    return {key: value for key, value in section.items()
            if key not in ('nav_event_timeout', 'show_logs', 'record_bag')}


def load_headless_executor_section(path: Path) -> dict:
    """Read the existing headless executor section without changing its file format.

    Args:
        path: Existing headless YAML file, opened read-only.

    Returns:
        Executor options, excluding the three launcher-only settings.

    Raises:
        ConfigError: The file cannot be read or lacks a mapping executor section.
    """
    return _executor_section(_read_mapping(path), path)


def _profile_values(path):
    profile = _read_mapping(path)
    bindings = {
        'robot_base_frame': ('frames', 'poi_capture'),
        'map_frame': ('frames', 'map'), 'odom_frame': ('frames', 'odom'),
        'odom_topic': ('nav2', 'odom_topic'),
        'grid_map_topic': ('executor', 'grid_map_topic'),
        'poi_store_file': ('executor', 'poi_store_file'),
    }
    values, sources = {}, {}
    for name, (section, key) in bindings.items():
        try:
            values[name] = profile[section][key]
        except (KeyError, TypeError) as error:
            raise ConfigError(f'{path}: missing {section}.{key}') from error
        sources[name] = f'profile:{path}:{section}.{key}'
    options = profile.get('executor', {})
    for key, value in options.items():
        name = 'reconnect_delay' if key == 'reconnect_delay_sec' else key
        values[name] = value
        sources[name] = f'profile:{path}:executor.{key}'
    return values, sources


def _normalize(values):
    definitions = {field.name: field for field in fields(ExecutorConfig)}
    unknown = set(values) - definitions.keys()
    if unknown:
        raise ConfigError(f'unknown executor options: {sorted(unknown, key=str)}')
    for name, value in tuple(values.items()):
        expected = definitions[name].type
        if expected in (int, float):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0
                    or (expected is int and not isinstance(value, int))):
                raise ConfigError(f'{name} must be a finite nonnegative {expected.__name__}')
            if name in ('heartbeat_interval', 'loop_closure_poll_sec', 'nav_goal_snap_step',
                        'cancel_timeout_sec') and value == 0:
                raise ConfigError(f'{name} must be positive')
            values[name] = expected(value)
        elif name == 'dock_poi_ids':
            if not isinstance(value, (list, tuple)) or not all(isinstance(v, str) for v in value):
                raise ConfigError('dock_poi_ids must be a list of strings')
            values[name] = tuple(dict.fromkeys(v.strip() for v in value if v.strip()))
        elif name in ('pose_file', 'source_poi_file') and value is None:
            continue
        elif not isinstance(value, str) or not value.strip():
            raise ConfigError(f'{name} must be a nonempty string')
        if name in ('poi_store_file', 'pose_file', 'source_poi_file') and value is not None:
            values[name] = os.path.expanduser(value)


def load_configuration(
    raw_args: Sequence[str], *, profile_path: Path,
    environ: Mapping[str, str] | None = None,
) -> ResolvedConfig:
    """Resolve CLI > NAV_WS_URL > requested YAML > robot data/dataclass defaults.

    Other environment variables do not override settings. Unknown CLI arguments
    are preserved for ROS. A --robot-profile override selects another robot's
    data without changing code; --config is a flat map or a headless executor section.

    Args:
        raw_args: Command-line arguments without executable name.
        profile_path: Default robot data file, normally selected by the outer root.
        environ: Environment view, or None to use the actual process environment.

    Returns:
        Frozen effective configuration, immutable source mapping and ROS arguments.

    Raises:
        ConfigError: A file, option, known flag or configuration value is invalid.
    """
    from nav_executor.app.cli import parse_arguments

    explicit, ros_args = parse_arguments(raw_args)
    if explicit.pop('help', False):
        raise ConfigError('handle --help before loading runtime configuration')
    profile_path = Path(explicit.pop('robot_profile', profile_path)).expanduser()
    values, sources = _profile_values(profile_path)
    config_path = explicit.pop('config', None)
    if config_path is not None:
        path = Path(config_path).expanduser()
        data = _read_mapping(path)
        overrides = _executor_section(data, path) if 'executor' in data else data
        values.update(overrides)
        sources.update({name: f'yaml:{path}' for name in overrides})
    environment = os.environ if environ is None else environ
    if 'NAV_WS_URL' in environment:
        values['server_uri'] = environment['NAV_WS_URL']
        sources['server_uri'] = 'env:NAV_WS_URL'
    values.update(explicit)
    sources.update({name: 'cli' for name in explicit})
    _normalize(values)
    config = ExecutorConfig(**values)
    return ResolvedConfig(config, MappingProxyType({
        field.name: sources.get(field.name, 'default') for field in fields(ExecutorConfig)}),
        tuple(ros_args))
