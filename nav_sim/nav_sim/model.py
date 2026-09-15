"""ROS-free planar integration retaining the legacy fake chassis Euler rule."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Tuple


@dataclass(frozen=True)
class PlanarPose:
    """Planar pose in the configured odometry frame.

    Attributes:
        x: Position in metres.
        y: Position in metres.
        yaw: Heading in radians.
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def finite_number(value: float, name: str) -> float:
    """Convert a finite numeric value without silently accepting booleans.

    Args:
        value: Numeric value.
        name: Parameter name for diagnostics.

    Returns:
        Value as a float.

    Raises:
        ValueError: Value is not a finite number.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    if not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def integrate(pose: PlanarPose, linear: float, angular: float, dt: float) -> PlanarPose:
    """Take one explicit Euler step, exactly as sim_fake_chassis.py does.

    Args:
        pose: Previous pose.
        linear: Forward speed in metres per second; no lateral integration.
        angular: Yaw rate in radians per second.
        dt: Nonnegative fixed time step in seconds.

    Returns:
        New immutable pose with normalized yaw.

    Raises:
        ValueError: Any input is nonfinite or dt is negative.
    """
    x, y, yaw, linear, angular, dt = (
        finite_number(value, name) for name, value in (
            ('x', pose.x), ('y', pose.y), ('yaw', pose.yaw),
            ('linear', linear), ('angular', angular), ('dt', dt)))
    if dt < 0:
        raise ValueError('dt must be nonnegative')
    heading = yaw + angular * dt
    return PlanarPose(x + linear * math.cos(yaw) * dt,
                      y + linear * math.sin(yaw) * dt,
                      math.atan2(math.sin(heading), math.cos(heading)))


def quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return a normalized planar quaternion in x, y, z, w order.

    Args:
        yaw: Finite heading in radians.

    Returns:
        Quaternion components.
    """
    yaw = finite_number(yaw, 'yaw')
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def require_sandbox(environment: Mapping[str, str]) -> None:
    """Refuse simulation entry points outside the reserved offline ROS domain.

    Args:
        environment: Process environment.

    Raises:
        RuntimeError: ROS_DOMAIN_ID is not explicitly 99.
    """
    if environment.get('ROS_DOMAIN_ID') != '99':
        raise RuntimeError('nav_sim requires explicit ROS_DOMAIN_ID=99')


def frame_name(value: str, parameter: str) -> str:
    """Validate a required robot-frame parameter without inventing a default.

    Args:
        value: Frame name supplied by a robot configuration.
        parameter: Parameter name for diagnostics.

    Returns:
        Validated frame name.

    Raises:
        ValueError: Name is empty, absolute or contains whitespace.
    """
    if (not isinstance(value, str) or not value or value.startswith('/')
            or any(char.isspace() for char in value)):
        raise ValueError(f'{parameter} must be a nonempty relative TF frame')
    return value


def localization_frames(map_frame: str, odom_frame: str, body_frame: str) -> Tuple[str, ...]:
    """Validate the two dynamic transform edges owned by fake localization.

    Args:
        map_frame: Global fixed frame.
        odom_frame: Integrated odometry frame.
        body_frame: Robot body frame, not the navigation rotation centre.

    Returns:
        Validated map, odometry and body frame names.

    Raises:
        ValueError: Names are invalid or would create a TF self-edge/cycle.
    """
    frames = tuple(frame_name(value, name) for name, value in (
        ('map_frame', map_frame), ('odom_frame', odom_frame), ('body_frame', body_frame)))
    if len(set(frames)) != 3:
        raise ValueError('map_frame, odom_frame and body_frame must be distinct')
    return frames
