"""Construct the convenience launch against Humble without running any process."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions
import pytest


def _module():
    path = Path(__file__).resolve().parents[1] / 'launch/keyboard_teleop.launch.py'
    spec = importlib.util.spec_from_file_location('keyboard_entry', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


def test_launch_constructs_one_recorded_owner_with_overrides(monkeypatch):
    """Selectors become shell-free up argv, never three competing ROS node actions."""
    module = _module()
    monkeypatch.setattr(module, 'resolve_executable', lambda *_: '/fake/navrun')
    context = _context(module, robot='example_diffdrive', start_driver='off',
                       layout='uiojkl', linear_speed='0.1', angular_speed='0.2',
                       repo='/private/path with spaces', dry_run='true')
    actions = module._setup(context)
    owners = [action for action in actions if isinstance(action, ExecuteProcess)]
    assert len(actions) == 2 and len(owners) == 1
    command = [perform_substitutions(context, item)
               for item in owners[0].process_description.cmd]
    assert command == [
        '/fake/navrun', 'up', '--mode', 'teleop', '--robot', 'example_diffdrive',
        '--chassis', 'off', '--record', 'light', '--layout', 'uiojkl',
        '--linear-speed', '0.1', '--angular-speed', '0.2',
        '--repo', '/private/path with spaces', '--dry-run']


def test_launch_defaults_keep_profile_gears():
    """Unspecified gears are not replaced by unrelated launch defaults."""
    context = _context(_module())
    assert context.launch_configurations['linear_speed'] == ''
    assert context.launch_configurations['angular_speed'] == ''
    assert context.launch_configurations['layout'] == 'wasd'


@pytest.mark.parametrize('value', ['nan', 'inf', '-1', 'not-a-number'])
def test_launch_rejects_bad_gears_without_resolving_executable(monkeypatch, value):
    """Malformed user selectors fail before any command is constructed."""
    module = _module()
    monkeypatch.setattr(module, 'resolve_executable', lambda *_: pytest.fail('resolved'))
    with pytest.raises(ValueError):
        module._setup(_context(module, linear_speed=value))


@pytest.mark.parametrize('code', [2, 130, -9])
def test_failed_child_cannot_appear_as_success(code):
    """Launch receives an error instead of silently succeeding after a failed child."""
    with pytest.raises(RuntimeError, match=f'exit {code}'):
        _module()._exited(SimpleNamespace(returncode=code), LaunchContext())


def test_quit_shuts_down_outer_launch():
    """A successful q exit ends the convenience launch after navrun finalization."""
    assert len(_module()._exited(SimpleNamespace(returncode=0), LaunchContext())) == 1
