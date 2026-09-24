"""Keyboard decisions and shaping without ROS or TTY I/O."""

from dataclasses import replace

import pytest

from chassis_teleop.control import KeyboardState, TeleopConfig, motion_for_key, shape_motion


@pytest.mark.parametrize('config', [
    TeleopConfig(), TeleopConfig(linear_speed=0.1, angular_speed=0.2),
    TeleopConfig(linear_deadzone=0.0, angular_deadzone=0.0),
])
def test_shaping_applies_speed_deadzone_and_ceiling(config):
    """Direction inputs are scaled, lifted through deadzones and bounded."""
    assert shape_motion(0.0, 0.0, config) == (0.0, 0.0)
    linear, angular = shape_motion(1.0, -1.0, config)
    assert 0.0 < linear <= config.max_linear_x
    assert -config.max_angular_z <= angular < 0.0


@pytest.mark.parametrize('key,motion', [
    ('w', (1, 0)), ('W', (1, 0)), ('s', (-1, 0)), ('a', (0, 1)), ('d', (0, -1)),
    ('\x1b[A', (1, 0)), ('\x1b[B', (-1, 0)), ('\x1b[C', (0, -1)),
    ('\x1b[D', (0, 1)), ('z', None), ('', None), ('q', None),
])
def test_default_key_map(key, motion):
    """WASD and arrows map to bounded differential-drive directions."""
    assert motion_for_key(key, 'wasd') == motion


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


def test_disabled_timeout_and_frequency_floor():
    """Disabling deadman timeout remains explicit and does not enable speed changes."""
    state = KeyboardState(TeleopConfig(key_timeout=0.0, publish_hz=0.0))
    state.handle_key('w', 1.0)
    assert state.command(100.0) == (0.2, 0.0)
    assert state.config.period_sec == 1.0


def test_command_lease_expires_after_the_configured_boundary():
    """A motion remains live through the timeout boundary and stops afterwards."""
    state = KeyboardState(TeleopConfig())
    state.handle_key('w', 10.0)
    assert state.command(10.4) == (0.2, 0.0)
    assert state.command(10.400001) == (0.0, 0.0)
