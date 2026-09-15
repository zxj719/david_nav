"""The startup CLI exposes an honest read-only preview during staged integration."""

import json
import os
from pathlib import Path

from nav_bringup.navrun import cli, up
from nav_bringup.navrun.preflight import ProcessInfo


SOURCE = Path(__file__).resolve().parents[2]


def test_up_parser_defaults():
    """The documented five-mode entry uses conservative, explicit selectors."""
    args = cli.build_parser().parse_args(['up', '--mode', 'precise', '--dry-run'])
    assert (args.robot, args.site, args.record, args.chassis) == (
        'david', 'daolan', 'light', 'auto')


def test_non_dry_run_is_rejected_before_discovery(monkeypatch, capsys):
    """Incomplete runtime integration cannot change platform settings or start hardware."""
    monkeypatch.setattr(up, 'preview', lambda _: (_ for _ in ()).throw(AssertionError()))
    assert cli.main(['up', '--mode', 'chassis']) == 2
    assert 'not yet enabled' in capsys.readouterr().err


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
    assert report['runtime_enabled'] is False
    assert report['errors'] == []
    assert not (tmp_path / 'runs').exists()
