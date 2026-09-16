"""Navigation health policy is deterministic, bounded and independent of ROS/IMU."""

import math

import pytest


def _core(**changes):
    from nav_perception_iface.health_core import HealthMonitor, HealthPolicy

    return HealthMonitor(HealthPolicy(**changes))


def _pose(stamp, x=0, y=0, yaw=0):
    from nav_perception_iface.health_core import PoseSample

    return PoseSample(stamp, x, y, yaw)


def test_waiting_is_not_failure_and_ready_samples_need_no_imu():
    """Missing manual localization clears history but never creates motion authority."""
    core = _core()
    report = core.sample(1, False, None, None)
    assert report.code == 'localization_waiting' and report.severity == 'warn'
    report = core.sample(2, True, _pose(2), _pose(2))
    assert report.severity == 'info' and report.values['window_sec'] == 0


def test_static_wheel_odometry_and_28_metre_map_jump_are_an_error():
    """The observed historical failure must be visible without waiting for a full window."""
    core = _core()
    core.sample(1, True, _pose(1), _pose(1))
    report = core.sample(1.2, True, _pose(1.2, 28), _pose(1.2))
    assert report.code == 'localization_divergence' and report.severity == 'error'
    assert report.values['divergence_m'] == 28


def test_initial_frame_rotation_and_origin_do_not_create_false_divergence():
    """Compare anchored displacement vectors, not absolute positions in different frames."""
    core = _core()
    core.sample(1, True, _pose(1, 100, 200, math.pi / 2), _pose(1, 4, 5))
    report = core.sample(2, True, _pose(2, 100, 201, math.pi / 2), _pose(2, 5, 5))
    assert report.severity == 'info'
    assert report.values['divergence_m'] == pytest.approx(0, abs=1e-10)


def test_equal_lengths_in_opposite_directions_still_diverge():
    """A scalar distance-only comparison would miss a direction error."""
    core = _core()
    core.sample(1, True, _pose(1), _pose(1))
    report = core.sample(2, True, _pose(2, 1), _pose(2, -1))
    assert report.severity == 'error' and report.values['divergence_m'] == 2


@pytest.mark.parametrize('source', ['map', 'wheel'])
def test_stale_or_future_inputs_warn_and_break_the_comparison_window(source):
    """An outage cannot be bridged as if both samples were contemporaneous."""
    core = _core()
    core.sample(1, True, _pose(1), _pose(1))
    poses = {'map': _pose(2), 'wheel': _pose(2)}
    poses[source] = _pose(1)
    report = core.sample(2, True, poses['map'], poses['wheel'])
    assert report.severity == 'warn' and 'stale' in report.code
    report = core.sample(3, True, _pose(3, 20), _pose(3))
    assert report.values['window_sec'] == 0
    poses[source] = _pose(10)
    assert core.sample(3, True, poses['map'], poses['wheel']).severity == 'warn'


def test_window_expires_old_disagreement_and_uses_strict_thresholds():
    """Exactly the configured threshold is not an exceedance; old jumps eventually expire."""
    core = _core()
    core.sample(0, True, _pose(0), _pose(0))
    assert core.sample(1, True, _pose(1, 1), _pose(1)).severity == 'info'
    assert core.sample(2, True, _pose(2, 2), _pose(2)).severity == 'error'
    assert core.sample(3, True, _pose(3, 2), _pose(3)).severity == 'info'
    assert core.sample(5.5, True, _pose(5.5, 20), _pose(5.5)).values['window_sec'] == 0


def test_clock_regression_warns_and_restarts_history():
    """ROS/simulation clock reset must not join incompatible epochs."""
    core = _core()
    core.sample(10, True, _pose(10), _pose(10))
    report = core.sample(1, True, _pose(1, 20), _pose(1))
    assert report.code == 'localization_clock_reset' and report.severity == 'warn'
    assert core.sample(2, True, _pose(2, 20), _pose(2)).values['window_sec'] == 0


@pytest.mark.parametrize('key,value', [
    ('max_tf_age_sec', 0), ('max_tf_age_sec', float('nan')),
    ('max_divergence_m', -1), ('divergence_window_sec', True),
    ('divergence_window_sec', float('inf')), ('action', 'stop'), ('action', 'unknown'),
])
def test_invalid_policy_and_uncommissioned_stop_are_not_silently_accepted(key, value):
    """TODO-H1 remains explicitly uncommissioned; it must not silently become warn or stop."""
    with pytest.raises(ValueError):
        _core(**{key: value})


@pytest.mark.parametrize('value', [float('nan'), float('inf'), True])
def test_invalid_measurements_are_rejected(value):
    """No nonfinite value may reach the diagnostic JSON or the history buffer."""
    with pytest.raises(ValueError):
        _pose(1, value)


def test_history_is_bounded_and_duplicate_ticks_do_not_extend_it():
    """A large configured window cannot create an unbounded diagnostic buffer."""
    core = _core(divergence_window_sec=100000)
    for stamp in range(5000):
        core.sample(stamp, True, _pose(stamp), _pose(stamp))
    assert core.history_size <= 4096
    previous = core.history_size
    for _ in range(10):
        core.sample(4999, True, _pose(4999), _pose(4999))
    assert core.history_size == previous


def test_irregular_sampling_never_compares_outside_configured_window():
    """A sample just older than the window cannot extend it to a longer arbitrary interval."""
    core = _core()
    core.sample(0, True, _pose(0), _pose(0))
    core.sample(1.5, True, _pose(1.5), _pose(1.5))
    report = core.sample(2.1, True, _pose(2.1), _pose(2.1))
    assert 0 <= report.values['window_sec'] <= 2.0


def test_finite_extremes_cannot_produce_nonfinite_evidence():
    """Subtracting valid floats can overflow; the report must still be serializable."""
    core = _core()
    report = core.sample(1e308, True, _pose(-1e308), _pose(1e308))
    assert report.severity == 'warn'
    assert all(math.isfinite(value) for value in report.values.values())
