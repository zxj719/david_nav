"""Verify installed live checks against owned fake nodes, never a real navigation stack."""

from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_prefix
import pytest

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import schema
from nav_bringup.navrun.preparation import BundleInputs, prepare_bundle, write_bundle


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
    ('good', 0), ('bad', 1), ('waiting', 3), ('interrupt', 130)])
def test_installed_live_checker_read_only_status_and_cleanup(
        sessions, tmp_path, variant, expected):
    """Real DDS discovery/GetParameters/TF are exercised without sending any velocity."""
    root = _bundle(tmp_path)
    helper = Path(__file__).with_name('live_check_fixture.py')
    options = []
    if variant == 'bad':
        options.append('--bad-parameter')
    if variant in ('waiting', 'interrupt'):
        options.append('--no-localization')
    fixture = sessions.start('live_fixture', [
        sys.executable, str(helper), '--run-dir', str(root), *options])
    _ready(fixture, tmp_path / 'process_logs/live_fixture.log')
    executable = Path(get_package_prefix('nav_bringup')) / 'lib/nav_bringup/navcfg'
    checker = sessions.start('live_checker', [
        str(executable), 'check', '--live', '--strict', '--run-dir', str(root),
        '--wait-timeout', '8', '--events', str(tmp_path / 'events.jsonl')])
    path = tmp_path / 'process_logs/live_checker.log'
    if variant == 'interrupt':
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and checker.poll() is None:
            if 'localization_waiting' in path.read_text():
                break
            time.sleep(0.05)
        assert 'localization_waiting' in path.read_text(), path.read_text()
        checker.send_signal(signal.SIGINT)
    assert checker.wait(timeout=20) == expected, path.read_text()
    assert fixture.poll() is None, 'checker must not stop or signal external localization'
    output = path.read_text()
    if variant == 'good':
        assert '0 error(s)' in output
        assert 'localization_ready' in output
    elif variant == 'bad':
        assert 'parameter_mismatch' in output
    else:
        assert 'localization_waiting' in output
        assert 'localization_ready' not in output
