"""Compare keyboard decisions and shaping with legacy code without ROS or TTY I/O."""

import ast
from dataclasses import asdict, replace
import math
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

from chassis_teleop.control import KeyboardState, TeleopConfig, motion_for_key, shape_motion


SOURCE = Path(__file__).resolve().parents[2]


def _legacy(config):
    tree = ast.parse((SOURCE / 'david_cmd/david_cmd/david_keyboard.py').read_text())
    tree.body = [node for node in tree.body if isinstance(
        node, (ast.Assign, ast.FunctionDef, ast.ClassDef))]
    scope = {'Node': object, 'math': math, 'Twist': lambda: SimpleNamespace(
        linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))}
    exec(compile(tree, '<legacy-keyboard-no-imports>', 'exec'), scope)
    node = object.__new__(scope['DavidKeyboardNode'])
    for key, value in asdict(config).items():
        setattr(node, key, value)
    return node, scope


@pytest.mark.parametrize('config', [
    TeleopConfig(), TeleopConfig(linear_speed=0.1, angular_speed=0.2),
    TeleopConfig(linear_deadzone=0.0, angular_deadzone=0.0),
])
def test_shaping_is_equal_to_actual_legacy_methods(config):
    """Finite direction inputs match old clamp, signed deadzone and ceiling ordering."""
    legacy, _ = _legacy(config)
    rng = random.Random(44)
    samples = [(x, yaw) for x in (-1.0, 0.0, 1.0) for yaw in (-1.0, 0.0, 1.0)]
    samples.extend((rng.uniform(-2, 2), rng.uniform(-2, 2)) for _ in range(500))
    for x, yaw in samples:
        expected = legacy.build_twist(x, yaw)
        assert shape_motion(x, yaw, config) == (expected.linear.x, expected.angular.z)


@pytest.mark.parametrize('key', ['w', 'W', 'a', 's', 'd', '\x1b[A', '\x1b[B',
                                      '\x1b[C', '\x1b[D', 'z', '', 'q'])
def test_default_key_map_matches_legacy(key):
    """WASD and arrows retain their exact direction and case behavior."""
    _, legacy = _legacy(TeleopConfig())
    assert motion_for_key(key, 'wasd') == legacy['motion_for_key'](key)


@pytest.mark.parametrize('key,motion', [
    ('u', (1, 1)), ('i', (1, 0)), ('o', (1, -1)), ('j', (0, 1)), ('l', (0, -1)),
    ('m', (-1, -1)), (',', (-1, 0)), ('.', (-1, 1)), ('w', None),
])
def test_alternate_layout_is_bounded_differential_motion(key, motion):
    """The alternate layout adds arcs, not lateral movement or speed-increase keys."""
    assert motion_for_key(key, 'uiojkl') == motion


def test_hold_timeout_ignored_keys_stop_and_quit():
    """Only motion keys refresh the lease; stop and quit are explicit control events."""
    state = KeyboardState(TeleopConfig())
    assert state.command(10.0) == (0.0, 0.0)
    assert state.handle_key('w', 10.0) == 'motion'
    assert state.command(10.39) == (0.2, 0.0)
    assert state.handle_key('?', 10.3) == 'ignored'
    assert state.command(10.401) == (0.0, 0.0)
    state.handle_key('a', 11.0)
    assert state.command(11.0) == (0.0, 0.35)
    assert state.handle_key('k', 11.1) == 'stop'
    assert state.command(11.1) == (0.0, 0.0)
    assert state.handle_key('q', 11.2) == 'quit'


@pytest.mark.parametrize('key', [' ', 'k', 'K'])
def test_stop_keys_always_reset_state(key):
    """A stop does not leave a delayed motion command behind."""
    state = KeyboardState(TeleopConfig())
    state.handle_key('w', 1.0)
    assert state.handle_key(key, 1.1) == 'stop'
    assert state.command(1.1) == (0.0, 0.0)


@pytest.mark.parametrize('key', ['q', 'Q', '\x03'])
def test_quit_keys_are_not_speed_adjustments(key):
    """Legacy quit aliases remain quit in both layouts."""
    for layout in ('wasd', 'uiojkl'):
        assert KeyboardState(TeleopConfig(layout=layout)).handle_key(key, 1.0) == 'quit'


@pytest.mark.parametrize('key,value', [
    ('linear_speed', float('nan')), ('angular_speed', float('inf')),
    ('linear_deadzone', -1.0), ('layout', 'omni'), ('cmd_vel_topic', ''), ('tty_device', ''),
])
def test_invalid_configuration_is_rejected(key, value):
    """Malformed configuration cannot reach an eventual publisher."""
    with pytest.raises(ValueError):
        KeyboardState(replace(TeleopConfig(), **{key: value}))


def test_legacy_disabled_timeout_and_frequency_floor():
    """Disabling deadman timeout remains explicit and does not enable speed changes."""
    state = KeyboardState(TeleopConfig(key_timeout=0.0, publish_hz=0.0))
    state.handle_key('w', 1.0)
    assert state.command(100.0) == (0.2, 0.0)
    assert state.config.period_sec == 1.0


def test_command_lease_matches_legacy_across_floating_time_boundaries():
    """Monotonic subtraction remains identical, including representational edge cases."""
    config = TeleopConfig()
    old, scope = _legacy(config)
    state = KeyboardState(config)
    state.handle_key('w', 10.0)
    old.last_motion_time = 10.0
    old.active_twist = old.build_twist(1.0, 0.0)
    for now in (10.0, 10.39, 10.4, 10.400001, 11.0):
        scope['time'] = SimpleNamespace(monotonic=lambda: now)
        expected = old.current_command()
        assert state.command(now) == (expected.linear.x, expected.angular.z)
