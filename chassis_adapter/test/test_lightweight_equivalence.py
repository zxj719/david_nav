"""In-memory command equivalence and explicit timing deltas for David L2/L3 presets.

No ROS node, publisher, goal or serial port is created. Steady command equivalence
is intentionally limited to differential (y=0) inputs inside the upstream contract;
hardware timing, braking and out-of-contract input behavior require separate review.
"""

from dataclasses import replace
from pathlib import Path

import pytest

from chassis_adapter.core import AdapterConfig, AdapterState, shape_command
from nav_bringup.navcfg import render


SOURCE = Path(__file__).resolve().parents[2]
DAVID = SOURCE / 'david_config'


def _config(mode, experiment=None):
    path = DAVID / 'modes' / f'{mode}.yaml' if experiment is None else (
        DAVID / 'modes/experiments' / f'{mode}_{experiment}.yaml')
    overlay = render.parse_mode(render.load_yaml(path), path)
    effective = render.deep_merge(render.load_yaml(DAVID / 'robot/david.yaml'), overlay.profile)
    params = effective['chassis']['adapter']
    return AdapterConfig(
        input_cmd_vel_topic=params['input_topic'],
        output_cmd_vel_topic=effective['chassis']['cmd_topic'],
        cmd_vel_timeout=params['cmd_timeout_sec'], min_linear_x=params['min_linear_x'],
        max_linear_x=params['hard_max_linear_x'], min_angular_z=params['min_angular_z'],
        min_stationary_angular_z=params['min_stationary_angular_z'],
        stationary_angular_threshold=params['stationary_angular_threshold'],
        max_angular_z=params['hard_max_angular_z'], publish_hz=params['publish_hz'])


@pytest.mark.parametrize('mode', ['tour', 'precise'])
@pytest.mark.parametrize('experiment', ['l2', 'l3', 'l2_l3'])
def test_valid_differential_commands_keep_steady_shaping(mode, experiment):
    """Navigation/back-up commands and capped Spin retain the 0.55 stationary floor."""
    before, after = _config(mode), _config(mode, experiment)
    assert replace(after, cmd_vel_timeout=before.cmd_vel_timeout,
                   publish_hz=before.publish_hz) == before
    smoother = render.load_yaml(SOURCE / 'nav_bringup/config/nav2_base.yaml')[
        'velocity_smoother']['ros__parameters']
    assert smoother['feedback'] == 'OPEN_LOOP'
    assert smoother['deadband_velocity'] == [0.0, 0.0, 0.0]
    upper, lower = smoother['max_velocity'], smoother['min_velocity']
    for axis in (0, 2):
        # At nominal tick spacing, even a full reversal is below the accel cap.
        assert smoother['max_accel'][axis] / smoother['smoothing_frequency'] >= (
            upper[axis] - lower[axis])
        assert -smoother['max_decel'][axis] / smoother['smoothing_frequency'] >= (
            upper[axis] - lower[axis])
    for x_index in range(-20, 21):
        for yaw_index in range(-50, 51):
            x, yaw = x_index / 100, yaw_index / 100
            smoothed_x = max(lower[0], min(upper[0], x))
            smoothed_yaw = max(lower[2], min(upper[2], yaw))
            assert shape_command(x, 0.0, yaw, after) == shape_command(
                smoothed_x, 0.0, smoothed_yaw, before)
    for yaw in (-0.8, 0.8):
        old_spin = max(lower[2], min(upper[2], yaw))
        new_spin = max(-0.5, min(0.5, yaw)) if 'l2' in experiment else old_spin
        assert shape_command(0.0, 0.0, old_spin, before) == shape_command(
            0.0, 0.0, new_spin, after)
        assert abs(shape_command(0.0, 0.0, new_spin, after)[1]) == 0.55


@pytest.mark.parametrize('experiment,timeout,hz', [
    ('l2', 1.0, 50.0), ('l3', 2.0, 20.0), ('l2_l3', 1.0, 20.0)])
def test_timeout_and_timer_changes_are_explicit_not_claimed_identical(experiment, timeout, hz):
    """The first stale tick sends zero once; rate changes do not alter command shaping."""
    config = _config('tour', experiment)
    assert config.cmd_vel_timeout == timeout
    assert config.period_sec == 1.0 / hz
    state = AdapterState(config, 0)
    state.receive(0.1, 0.0, 0.0, 0)
    threshold = int(timeout * 1e9)
    assert state.output(threshold) == (0.1, 0.0)
    assert state.output(threshold + 1) == (0.0, 0.0)
    assert state.output(threshold + 2) is None
