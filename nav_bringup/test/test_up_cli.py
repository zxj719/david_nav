"""The startup CLI dispatches live work explicitly and keeps previews read-only."""

import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from nav_bringup.navrun import cli, up
from nav_bringup.navrun.preflight import ProcessInfo


SOURCE = Path(__file__).resolve().parents[2]


def test_up_parser_defaults():
    """The four-mode entry uses conservative, explicit selectors."""
    args = cli.build_parser().parse_args(['up', '--mode', 'precise', '--dry-run'])
    assert (args.robot, args.site, args.record, args.chassis) == (
        'david', 'daolan', 'light', 'auto')
    assert (args.layout, args.linear_speed, args.angular_speed) == (None, None, None)


def test_up_parser_keyboard_overrides():
    """Convenience launch and direct up share exactly the same override selectors."""
    args = cli.build_parser().parse_args([
        'up', '--mode', 'teleop', '--layout', 'uiojkl',
        '--linear-speed', '0.1', '--angular-speed', '0.2'])
    assert (args.layout, args.linear_speed, args.angular_speed) == ('uiojkl', 0.1, 0.2)


def test_live_cli_dispatches_without_using_preview(monkeypatch):
    """The live branch delegates to recorded startup; this test never starts it."""
    received = []
    monkeypatch.setattr(up, 'preview', lambda _: (_ for _ in ()).throw(AssertionError()))
    monkeypatch.setattr(up, 'start', lambda args: received.append(args) or 143, raising=False)
    assert cli.main(['up', '--mode', 'chassis', '--strict']) == 143
    assert received[0].strict is True


def test_dry_run_cli_uses_real_source_profiles(tmp_path, monkeypatch, capsys):
    """CLI rendering requires no ROS node, child process, lock or run directory."""
    monkeypatch.setattr(up, '_processes', lambda: (
        ProcessInfo(os.getpid(), 1, 1.0, ('navrun',), os.getuid()),))
    monkeypatch.setattr(up, 'resolve_executable', lambda *_: '/not-executed')
    result = cli.main([
        'up', '--mode', 'tour', '--robot', 'example_diffdrive', '--site', 'offline',
        '--no-executor', '--dry-run', '--source-root', str(SOURCE),
        '--runs-root', str(tmp_path / 'runs')])
    assert result == 0
    report = json.loads(capsys.readouterr().out)
    assert report['dry_run'] is True
    assert report['runtime_enabled'] is True
    assert report['errors'] == []
    assert not (tmp_path / 'runs').exists()


@pytest.mark.parametrize('mode,expected_site', [('chassis', 'none'), ('precise', 'offline')])
def test_start_binds_recorded_options_and_callbacks(tmp_path, monkeypatch, mode, expected_site):
    """The real start function only runs through its injected recording boundary."""
    calls = []
    plan = object()
    commands = object()
    steps = object()
    monkeypatch.setattr(up, 'DEFAULT_LOCK_PATH', tmp_path / 'private.lock')
    monkeypatch.setattr(up, '_plan', lambda args, path: calls.append(path) or plan)
    monkeypatch.setattr(up, 'bind_commands', lambda value, **kwargs: (
        calls.append((value, kwargs['strict'])) or commands))
    monkeypatch.setattr(up, 'bind_host_steps', lambda path: calls.append(path) or steps)

    def recorded(options, prepare, bind, host):
        assert (options.mode, options.robot, options.site) == (
            mode, 'example_diffdrive', expected_site)
        assert options.lock_path == tmp_path / 'private.lock'
        assert options.repo_dir == tmp_path / 'repository'
        assert options.runs_root == tmp_path / 'runs'
        assert options.command[-1] == '--strict'
        assert host is steps
        assert prepare(tmp_path / 'run') is plan
        assert bind(plan) is commands
        return SimpleNamespace(run=SimpleNamespace(path=tmp_path / 'run'),
                               exit_code=130, exit_reason='signal_2')

    monkeypatch.setattr(up, 'run_recorded_startup', recorded)
    assert cli.main([
        'up', '--mode', mode, '--robot', 'example_diffdrive', '--site', 'offline',
        '--repo', str(tmp_path / 'repository'), '--runs-root', str(tmp_path / 'runs'),
        '--strict']) == 130
    assert calls == [Path('/dev/shm'), tmp_path / 'run', (plan, True)]
    assert not (tmp_path / 'runs').exists()


def test_start_without_repository_fails_before_host_binding(tmp_path, monkeypatch, capsys):
    """An installed layout cannot silently fall back to a production source tree."""
    monkeypatch.setattr(up, '__file__', str(tmp_path / 'a/b/c/d/up.py'))
    monkeypatch.setattr(up, 'bind_host_steps', lambda _: pytest.fail('host binding reached'))
    assert cli.main(['up', '--mode', 'chassis']) == 2
    assert 'provide --repo explicitly' in capsys.readouterr().err
