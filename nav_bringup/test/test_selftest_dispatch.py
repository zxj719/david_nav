"""Optional selftest CLI registration without a nav_sim dependency cycle."""

import sys
from types import SimpleNamespace

from nav_bringup.navrun import cli


def test_parser_does_not_import_optional_simulator(monkeypatch):
    """Normal navrun commands and help remain usable with nav_sim absent."""
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest', None)
    args = cli.build_parser().parse_args(['selftest', '--mode', 'tour', '--prepare-only'])
    assert args.robot == 'david' and args.prepare_only
    assert not args.startup_only and not args.allow_goals
    assert args.timeout == 180
    assert cli.build_parser().parse_args(['doctor']).subcommand == 'doctor'


def test_missing_optional_feature_has_clear_nonzero_exit(monkeypatch, capsys):
    """An absent optional package is a visible error, never a successful no-op."""
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest', None)
    assert cli.main(['selftest', '--mode', 'precise', '--prepare-only']) == 2
    assert 'optional nav_sim package' in capsys.readouterr().err


def test_cli_dispatch_preserves_requested_mode_and_nonzero_status(monkeypatch):
    """Only the explicitly selected optional command imports its implementation."""
    seen = []
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest',
                        SimpleNamespace(run=lambda args: seen.append(args) or 7))
    assert cli.main(['selftest', '--mode', 'precise', '--startup-only']) == 7
    assert seen[0].mode == 'precise' and seen[0].startup_only
    assert not seen[0].allow_goals
