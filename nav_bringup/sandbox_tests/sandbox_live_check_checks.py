"""Verify installed live checks against owned fake nodes, never a real navigation stack."""

import os
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_prefix
import psutil
import pytest

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import schema
from nav_bringup.navrun.preparation import BundleInputs, prepare_bundle, write_bundle
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import read_events
from conftest import require_sandbox


def _bundle(tmp_path):
    source = Path(__file__).resolve().parents[2]
    config = source / 'example_diffdrive_config'
    bringup = source / 'nav_bringup/config'
    plugins = source / 'nav_plugins'
    root = tmp_path / 'run'
    inputs = BundleInputs(
        config / 'robot/example_diffdrive.yaml', config / 'modes/tour.yaml',
        bringup / 'nav2_base.yaml', bringup / 'nav2_bindings.yaml',
        plugins / 'behavior_trees', plugins / 'config/bt_bindings.yaml',
        site=config / 'sites/offline/site.yaml', urdf=config / 'urdf/example_diffdrive.urdf')
    prepared = prepare_bundle(
        GraphOptions('tour', str(root), start_chassis=False, robot_state=False), inputs,
        schema.load_catalog(schema.find_source_schemas(source)))
    root.mkdir()
    write_bundle(prepared)
    return root


def _ready(process, path):
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline and process.poll() is None:
        if 'live fixture ready' in path.read_text():
            return
        time.sleep(0.05)
    pytest.fail(path.read_text())


@pytest.mark.parametrize('variant,expected', [
    ('good', 0), ('bad', 1), ('silent', 1), ('waiting', 3), ('interrupt', 130),
    ('interrupt_sample', 130)])
def test_installed_live_checker_read_only_status_and_cleanup(
        tmp_path, variant, expected):
    """Real DDS discovery/GetParameters/TF are exercised without sending any velocity."""
    require_sandbox()
    root = _bundle(tmp_path)
    helper = Path(__file__).with_name('live_check_fixture.py')
    options = []
    if variant == 'bad':
        options.append('--bad-parameter')
    if variant == 'silent':
        options.append('--silent-odom')
    if variant in ('waiting', 'interrupt'):
        options.append('--no-localization')
    fixture = checker = None
    sample_pid = None
    path = tmp_path / 'checker.log'
    try:
        fixture = OwnedProcess([
            sys.executable, str(helper), '--run-dir', str(root), *options],
            os.environ, tmp_path / 'fixture.log')
        _ready(fixture, tmp_path / 'fixture.log')
        executable = Path(get_package_prefix('nav_bringup')) / 'lib/nav_bringup/navcfg'
        checker = OwnedProcess([
            str(executable), 'check', '--live', '--strict', '--run-dir', str(root),
            '--wait-timeout', '8', '--sample-sec', '4' if variant == 'interrupt_sample' else '1',
            '--events', str(tmp_path / 'events.jsonl')],
            os.environ, path)
        if variant == 'interrupt':
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and checker.poll() is None:
                if 'localization_waiting' in path.read_text():
                    break
                time.sleep(0.05)
            assert 'localization_waiting' in path.read_text(), path.read_text()
            os.kill(checker.pid, signal.SIGINT)
        if variant == 'interrupt_sample':
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and checker.poll() is None:
                children = psutil.Process(checker.pid).children()
                if children:
                    assert len(children) == 1
                    sample_pid = children[0].pid
                    assert os.getpgid(sample_pid) == checker.pid
                    break
                time.sleep(0.05)
            assert sample_pid is not None, path.read_text()
            os.kill(checker.pid, signal.SIGINT)
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline and checker.poll() is None:
            time.sleep(0.05)
        assert checker.poll() == expected, path.read_text()
        assert fixture.poll() is None, 'checker must not stop or signal external localization'
        output = path.read_text()
        if variant == 'good':
            assert '0 error(s)' in output
            assert 'interface_contracts_ready' in output
            events = read_events(tmp_path / 'events.jsonl')[0]
            evidence = [event.data for event in events if event.event_type == 'interface_snapshot']
            assert len(evidence) == 1 and evidence[0]['status'] == 0
            assert evidence[0]['evidence']['samples']
        elif variant == 'bad':
            assert 'parameter_mismatch' in output
        elif variant == 'silent':
            assert 'localization_odom_missing' in output
        elif variant == 'interrupt_sample':
            assert not Path('/proc', str(sample_pid)).exists()
            assert 'interface_contracts_ready' not in output
        else:
            assert 'localization_waiting' in output
            assert 'localization_ready' not in output
    finally:
        if checker is not None:
            checker.close()
        if fixture is not None:
            fixture.close()
