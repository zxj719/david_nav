"""Preserve Dock ROS declarations while applying launch and ROS CLI overrides."""

from typing import Mapping

from nav_tasks.domain.runtime_config import DockNodeConfig
from nav_tasks.skills.docking.parameters import from_parameters, PARAMETER_NAMES


def declare_parameters(
    node: object, defaults: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """Declare the old sixteen parameters and validate their effective values.

    Defaults must be complete. Validation happens before the first declaration;
    ROS overrides are then checked before the caller constructs motion resources.
    The caller owns the node, including partially completed declarations on error.
    This function never opens ROS, subscribes, creates clients or starts a worker.

    Args:
        node: Externally owned node, named/remapped by the outer composition root.
        defaults: Explicit legacy parameter defaults, retaining the footprint string.
        spin_action_name: Backend endpoint supplied separately, not a new ROS parameter.
        pose_timeout_sec: TF lookup budget supplied separately, not a new ROS parameter.

    Returns:
        Effective immutable settings after launch/ROS argument overrides.

    Raises:
        ConfigError: Defaults or effective ROS values are invalid.
        Exception: Parameter declaration/read failed; caller retains the node for cleanup.
    """
    from_parameters(defaults, spin_action_name=spin_action_name,
                    pose_timeout_sec=pose_timeout_sec)
    defaults = dict(defaults)
    for name in PARAMETER_NAMES:
        node.declare_parameter(name, defaults[name])
    effective = {name: node.get_parameter(name).value for name in PARAMETER_NAMES}
    return from_parameters(effective, spin_action_name=spin_action_name,
                           pose_timeout_sec=pose_timeout_sec)
