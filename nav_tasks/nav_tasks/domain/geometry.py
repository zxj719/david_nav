"""Frame-explicit planar geometry with no ROS dependencies."""

import math

from nav_tasks.domain.types import PoseRecord, Transform2D


def wrap_to_pi(angle: float) -> float:
    """Normalize a finite angle to the half-open interval [-pi, pi).

    Args:
        angle: Angle in radians.

    Returns:
        Wrapped angle; both signed pi boundaries map to negative pi.

    Raises:
        ValueError: The angle is not finite.
    """
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw after scale-safe normalization of a nonzero quaternion.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        Yaw in radians in [-pi, pi).

    Raises:
        ValueError: Any component is nonfinite or all components are zero.
    """
    components = (x, y, z, w)
    if not all(math.isfinite(value) for value in components):
        raise ValueError('quaternion must be finite')
    scale = max(abs(value) for value in components)
    if scale == 0.0:
        raise ValueError('quaternion must be nonzero')
    scaled = tuple(value / scale for value in components)
    norm = math.sqrt(sum(value * value for value in scaled))
    x, y, z, w = (value / norm for value in scaled)
    return wrap_to_pi(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quaternion_from_yaw(yaw: float) -> dict[str, float]:
    """Return planar quaternion fields in legacy x/y/z/w serialization order.

    Args:
        yaw: Finite yaw in radians; not wrapped before conversion.

    Returns:
        Quaternion mapping matching PoseRecord.orientation for finite inputs.

    Raises:
        ValueError: The yaw is not finite.
    """
    if not math.isfinite(yaw):
        raise ValueError('yaw must be finite')
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw * 0.5), 'w': math.cos(yaw * 0.5)}


def legacy_yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Preserve the old pose-source arithmetic for wire/storage compatibility.

    Unlike yaw_from_quaternion, this migration helper does not normalize, reject
    zero/nonfinite inputs or wrap positive pi. New geometry should normally use
    the normalized helper; changing a migrated POI value needs a separate policy
    change because tiny rounding differences also alter unrounded JSON fields.

    Args:
        x: Legacy quaternion x component.
        y: Legacy quaternion y component.
        z: Legacy quaternion z component.
        w: Legacy quaternion w component.

    Returns:
        The exact legacy atan2 result.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_pose(tf: Transform2D, pose: PoseRecord, *, target_frame: str) -> PoseRecord:
    """Apply target-from-source planar transform without guessing frame names.

    Args:
        tf: Translation and rotation mapping the source into the target frame.
        pose: Source-frame pose; height and timestamp are retained.
        target_frame: Explicit name of the output coordinate frame.

    Returns:
        Transformed pose with normalized yaw.

    Raises:
        ValueError: Target frame is empty or geometry is nonfinite.
    """
    if not target_frame:
        raise ValueError('target_frame must be nonempty')
    if not all(math.isfinite(value) for value in (
            tf.x, tf.y, tf.yaw, pose.x, pose.y, pose.z, pose.yaw)):
        raise ValueError('transform and pose geometry must be finite')
    cosine, sine = math.cos(tf.yaw), math.sin(tf.yaw)
    return PoseRecord(
        target_frame, tf.x + cosine * pose.x - sine * pose.y,
        tf.y + sine * pose.x + cosine * pose.y, pose.z,
        wrap_to_pi(tf.yaw + pose.yaw), pose.stamp_sec)
