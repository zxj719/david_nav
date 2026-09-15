"""Pure geometry boundaries, quaternion normalization and explicit frame tests."""

import math

import pytest

from nav_tasks.domain.geometry import (
    quaternion_from_yaw, transform_pose, wrap_to_pi, yaw_from_quaternion,
)
from nav_tasks.domain.types import PoseRecord, Transform2D


@pytest.mark.parametrize('angle', [math.pi, -math.pi, 3 * math.pi, -3 * math.pi])
def test_pi_boundaries(angle):
    """Keep the same half-open signed pi boundary as the existing helpers."""
    assert wrap_to_pi(angle) == pytest.approx(-math.pi)


@pytest.mark.parametrize('yaw', [0.0, 0.25, -1.3, math.pi, -math.pi, 8.0])
@pytest.mark.parametrize('scale', [1.0, 17.0, -3.0, 1e300, 1e-300])
def test_quaternion_normalizes_arbitrary_finite_scale(yaw, scale):
    """Yaw is invariant under quaternion sign and very large/small scales."""
    quaternion = quaternion_from_yaw(yaw)
    pose = PoseRecord('world', 0.0, 0.0, 0.0, yaw, 0.0)
    assert quaternion == pose.orientation()
    actual = yaw_from_quaternion(*(value * scale for value in quaternion.values()))
    assert wrap_to_pi(actual - yaw) == pytest.approx(0.0, abs=1e-14)


def test_full_quaternion_not_only_planar_components():
    """Include roll and pitch terms when extracting a general quaternion yaw."""
    assert yaw_from_quaternion(0.5, 0.5, 0.5, 0.5) == pytest.approx(math.pi / 2)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_inputs_rejected(value):
    """Reject invalid geometry instead of silently returning unusable goals."""
    with pytest.raises(ValueError):
        wrap_to_pi(value)
    with pytest.raises(ValueError):
        quaternion_from_yaw(value)
    with pytest.raises(ValueError):
        yaw_from_quaternion(0, 0, value, 1)


def test_zero_quaternion_rejected():
    """A zero quaternion has no well-defined orientation."""
    with pytest.raises(ValueError, match='nonzero'):
        yaw_from_quaternion(0, 0, 0, 0)


def test_transform_preserves_height_stamp_and_requires_output_frame():
    """Apply target-from-source in the correct direction without guessing frames."""
    pose = PoseRecord('odom_fixture', 2, 1, 0.7, math.pi / 2, 123)
    result = transform_pose(Transform2D(10, 20, math.pi / 2), pose, target_frame='world')
    assert result.frame_id == 'world'
    assert (result.x, result.y, result.z) == pytest.approx((9, 22, 0.7))
    assert result.yaw == pytest.approx(-math.pi)
    assert result.stamp_sec == 123
    assert pose.frame_id == 'odom_fixture'
    with pytest.raises(TypeError):
        transform_pose(Transform2D(0, 0, 0), pose)
    with pytest.raises(ValueError):
        transform_pose(Transform2D(0, 0, 0), pose, target_frame='')
    with pytest.raises(ValueError):
        transform_pose(Transform2D(float('nan'), 0, 0), pose, target_frame='world')
