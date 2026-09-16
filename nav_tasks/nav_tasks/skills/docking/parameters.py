"""Bind the complete legacy Dock parameter mapping to immutable task settings."""

from typing import Mapping

from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.footprint import parse_footprint
from nav_tasks.domain.runtime_config import DockNodeConfig, DockRuntimeConfig
from nav_tasks.skills.docking.config import workflow_config

# Keep the old declaration order and names; no robot-specific defaults live here.
PARAMETER_NAMES = (
    'approach_dist', 'xy_ok', 'yaw_ok', 'max_yaw_retries', 'goal_topic', 'action_name',
    'robot_base_frame', 'global_frame', 'nav_timeout_sec', 'spin_timeout_sec',
    'static_map_topic', 'static_occupied_threshold', 'footprint', 'settle_max_rate_rad_s',
    'settle_window_sec', 'settle_timeout_sec',
)


def from_parameters(
    parameters: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """Validate all legacy values, without inventing missing robot or mode policy.

    Args:
        parameters: Exactly the sixteen existing Dock ROS parameters; footprint is a string.
        spin_action_name: Shared backend endpoint; the old client used the relative name spin.
        pose_timeout_sec: Explicit TF lookup budget, retaining the old two-second default.

    Returns:
        Immutable ingress and runtime values detached from the input mapping.

    Raises:
        ConfigError: A required value is missing, unknown, malformed or out of range.
    """
    if not isinstance(parameters, Mapping):
        raise ConfigError('Dock parameters must be a mapping')
    missing = set(PARAMETER_NAMES) - parameters.keys()
    unknown = parameters.keys() - set(PARAMETER_NAMES)
    if missing or unknown:
        raise ConfigError(f'Dock parameters: missing={sorted(missing)}, '
                          f'unknown={sorted(unknown, key=str)}')
    for name in ('goal_topic', 'action_name'):
        value = parameters[name]
        if not isinstance(value, str) or not value.strip():
            raise ConfigError(f'dock.{name} must be a nonempty string')
    if not isinstance(parameters['footprint'], str):
        raise ConfigError('dock.footprint must retain the ROS string parameter format')
    try:
        footprint = parse_footprint(parameters['footprint'])
    except ValueError as error:
        raise ConfigError(f'dock.footprint: {error}') from error
    values = {name: value for name, value in parameters.items()
              if name not in ('goal_topic', 'action_name', 'footprint')}
    runtime = DockRuntimeConfig(**values, footprint=footprint,
                                spin_action_name=spin_action_name,
                                pose_timeout_sec=pose_timeout_sec)
    workflow_config(runtime)
    return DockNodeConfig(parameters['goal_topic'], parameters['action_name'], runtime)
