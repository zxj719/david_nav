"""Compare the extracted adapter against unchanged legacy methods without ROS."""

import ast
from dataclasses import asdict, replace
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

from chassis_adapter.core import AdapterConfig, AdapterState, shape_command


SOURCE = Path(__file__).resolve().parents[2]


def _ns(seconds):
    return int(seconds * 1e9)


def _twist(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(linear=SimpleNamespace(x=x, y=y, z=0.0),
                           angular=SimpleNamespace(x=0.0, y=0.0, z=yaw))


def _legacy(config):
    tree = ast.parse((SOURCE / 'david_cmd/david_cmd/david_move.py').read_text())
    tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]
    scope = {'Node': object, 'Twist': _twist}
    exec(compile(tree, '<legacy-methods-only>', 'exec'), scope)
    node = object.__new__(scope['DavidMoveNode'])
    for key, value in asdict(config).items():
        setattr(node, key, value)
    return node


@pytest.mark.parametrize('config', [
    AdapterConfig(),
    AdapterConfig(min_linear_x=0.0, min_stationary_angular_z=0.55, max_angular_z=0.6),
    AdapterConfig(min_angular_z=0.2),
])
def test_sanitizing_matches_legacy_for_boundaries_and_random_commands(config):
    """Compare numerical outputs, including the raw lateral-input arc decision."""
    legacy = _legacy(config)
    samples = [(x, y, yaw) for x in (0.0, 1e-12, -0.2, 0.4)
               for y in (0.0, 0.1)
               for yaw in (0.0, 0.024999, 0.025, -0.025, 0.026, -0.8)]
    rng = random.Random(43)
    samples.extend((rng.uniform(-3, 3), 0.0, rng.uniform(-3, 3)) for _ in range(1000))
    for x, y, yaw in samples:
        expected = legacy.sanitize_twist(_twist(x, y, yaw))
        assert shape_command(x, y, yaw, config) == (expected.linear.x, expected.angular.z)
        assert expected.linear.y == expected.linear.z == 0.0


def test_initial_silence_timeout_boundary_and_resume():
    """Timeout remains strictly greater-than and zero is sent once per stale transition."""
    state = AdapterState(AdapterConfig(), _ns(100.0))
    assert state.output(_ns(100.0)) is None
    state.receive(0.1, 0.0, 0.0, _ns(100.0))
    assert state.output(_ns(102.0)) == (0.2, 0.0)
    assert state.output(_ns(102.0001)) == (0.0, 0.0)
    assert state.output(_ns(103.0)) is None
    state.receive(0.0, 0.0, 0.03, _ns(104.0))
    assert state.output(_ns(104.0)) == (0.0, 0.35)
    assert state.output(_ns(107.0)) == (0.0, 0.0)


@pytest.mark.parametrize('timeout', [0.0, -1.0])
def test_disabled_timeout_preserves_old_periodic_output(timeout):
    """A disabled timeout publishes the initial zero and continues the latest command."""
    state = AdapterState(replace(AdapterConfig(), cmd_vel_timeout=timeout), _ns(100.0))
    assert state.output(_ns(100.0)) == (0.0, 0.0)
    state.receive(0.1, 0.0, 0.0, _ns(100.0))
    assert state.output(_ns(10000.0)) == (0.2, 0.0)


def test_ros_clock_rewind_matches_legacy_age_semantics():
    """Migration does not silently replace the old ROS-time timeout with monotonic time."""
    state = AdapterState(AdapterConfig(), _ns(100.0))
    state.receive(0.1, 0.0, 0.0, _ns(100.0))
    assert state.output(_ns(90.0)) == (0.2, 0.0)


def test_diagnostics_report_timeout_and_output_without_changing_state():
    """Diagnostics never consume the one-shot stop or refresh input freshness."""
    state = AdapterState(AdapterConfig(), _ns(100.0))
    state.receive(0.1, 0.0, 0.03, _ns(100.0))
    report = state.diagnostics(_ns(103.0))
    assert report['input_count'] == 1
    assert report['timed_out'] is True
    assert report['output_linear_x'] == 0.0
    assert state.output(_ns(103.0)) == (0.0, 0.0)


def test_timeout_keeps_nanosecond_precision_at_epoch_sized_ros_times():
    """Subtract integers before seconds conversion, like rclpy Duration in the old node."""
    start = 1_789_482_000_123_456_789
    state = AdapterState(AdapterConfig(), start)
    state.receive(0.1, 0.0, 0.0, start)
    assert not state.is_stale(start + 2_000_000_000)
    assert state.is_stale(start + 2_000_000_001)


@pytest.mark.parametrize('key,value', [
    ('publish_hz', float('nan')), ('max_linear_x', -0.1),
    ('max_angular_z', float('inf')), ('input_cmd_vel_topic', ''),
    ('output_cmd_vel_topic', ''), ('min_linear_x', True),
])
def test_invalid_configuration_is_rejected(key, value):
    """Malformed limits or topics fail before a publisher can be constructed."""
    with pytest.raises(ValueError):
        replace(AdapterConfig(), **{key: value}).validate()


def test_frequency_floor_matches_legacy():
    """The effective timer frequency is at least one hertz, as before."""
    assert AdapterConfig(publish_hz=0.0).period_sec == 1.0
