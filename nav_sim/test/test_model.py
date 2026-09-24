"""Pure integration, coordinate and safety contracts."""

import math

import pytest

from nav_sim.model import frame_name, integrate, localization_frames, PlanarPose, quaternion
from nav_sim.model import require_sandbox


@pytest.mark.parametrize('linear,angular', [(0.0, 0.0), (0.2, 0.0), (-0.1, 0.7), (0.2, -2.0)])
def test_integrates_planar_velocity_for_one_hundred_ticks(linear, angular):
    """Repeated integration stays finite and the quaternion matches final yaw."""
    pose = PlanarPose()
    for _ in range(100):
        pose = integrate(pose, linear, angular, 0.02)
    assert all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))
    z, w = quaternion(pose.yaw)[2:]
    assert math.isclose(z * z + w * w, 1.0)


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), True, '0.1'])
def test_nonfinite_and_wrong_type_input_is_rejected(invalid):
    """Do not turn malformed commands into poisoned odometry."""
    with pytest.raises(ValueError):
        integrate(PlanarPose(), invalid, 0.0, 0.02)
    with pytest.raises(ValueError):
        integrate(PlanarPose(invalid), 0.0, 0.0, 0.02)


def test_integration_boundaries():
    """Zero dt is stable and yaw wraps across pi."""
    pose = PlanarPose(1.0, 2.0, 0.0)
    assert integrate(pose, 0.4, -0.3, 0.0) == pose
    assert integrate(PlanarPose(yaw=math.pi - 0.01), 0.0, 1.0, 0.02).yaw < 0
    with pytest.raises(ValueError):
        integrate(pose, 0.0, 0.0, -0.01)


@pytest.mark.parametrize('environment', [{}, {'ROS_DOMAIN_ID': '0'}, {'ROS_DOMAIN_ID': '98'}])
def test_simulation_requires_explicit_domain99(environment):
    """The fake tools cannot silently enter the production ROS domain."""
    with pytest.raises(RuntimeError, match='99'):
        require_sandbox(environment)
    require_sandbox({'ROS_DOMAIN_ID': '99'})


@pytest.mark.parametrize('invalid', ['', '/absolute', 'embedded space', None])
def test_invalid_frame_names(invalid):
    """Robot frames must be configured, not guessed."""
    with pytest.raises(ValueError):
        frame_name(invalid, 'body_frame')


def test_configurable_chain_rejects_self_edges():
    """A second robot can use distinct names without modifying any code."""
    assert localization_frames('world_2', 'encoder_2', 'body_2') == (
        'world_2', 'encoder_2', 'body_2')
    with pytest.raises(ValueError):
        localization_frames('world_2', 'body_2', 'body_2')
