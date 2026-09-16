"""CLI safety boundaries and offline artifact creation without ROS initialization."""

import json
from pathlib import Path
import sys

import pytest

from nav_bringup.navrun.cli import main


@pytest.mark.parametrize('flags', [[], ['--timeout', 'nan'], ['--timeout', '-1']])
def test_missing_authority_or_invalid_timeout_has_no_writes(tmp_path, monkeypatch, flags):
    """A plain selftest request never silently dispatches goals or starts ROS."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '99')
    output = tmp_path / 'run'
    assert main(['selftest', '--mode', 'tour', '--output-dir', str(output), *flags]) == 2
    assert not output.exists()


@pytest.mark.parametrize('switch', ['--allow-goals', '--startup-only'])
def test_non99_refuses_before_runtime_import(tmp_path, monkeypatch, switch):
    """An explicit start switch cannot bypass the hard domain guard."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest_runtime', None)
    output = tmp_path / 'run'
    assert main(['selftest', '--mode', 'precise', '--output-dir', str(output), switch]) == 2
    assert not output.exists()


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_prepare_only_creates_real_assets_but_not_runtime(tmp_path, monkeypatch, capsys, mode):
    """Offline preparation is explicitly reported as not executed and not complete."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setitem(sys.modules, 'nav_sim.selftest_runtime', None)
    output = tmp_path / 'run'
    arguments = ['selftest', '--mode', mode, '--robot', 'example_diffdrive',
                 '--source-root', str(Path(__file__).resolve().parents[2]),
                 '--output-dir', str(output), '--prepare-only']
    assert main(arguments) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['executed'] is result['complete'] is False
    assert result['kind'] == 'prepared'
    assert (output / 'selftest.json').is_file()
    assert not (output / 'result.json').exists()
    frozen = (output / 'selftest.json').read_bytes()
    assert main(arguments) == 2
    assert (output / 'selftest.json').read_bytes() == frozen


def test_execution_switches_are_mutually_exclusive(tmp_path):
    """A startup-only request cannot accidentally inherit permission to dispatch goals."""
    with pytest.raises(SystemExit) as error:
        main(['selftest', '--mode', 'tour', '--startup-only', '--allow-goals'])
    assert error.value.code == 2
