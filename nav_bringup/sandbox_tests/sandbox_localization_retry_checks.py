"""Interrupt and resume synthetic localization during a real installed full live check."""

import os
from pathlib import Path
import signal
import sys
import time

from ament_index_python.packages import get_package_prefix
import pytest

from conftest import require_sandbox
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import read_events
from sandbox_live_check_checks import _bundle, _ready


def _wait_text(process, path, text, timeout=20.0):
    deadline = time.monotonic() + timeout
    while process.poll() is None and time.monotonic() < deadline:
        if text in path.read_text():
            return
        time.sleep(0.05)
    pytest.fail(path.read_text())


@pytest.mark.parametrize('variant,expected', [('resume', 0), ('interrupt', 130), ('bounded', 3)])
def test_live_check_localization_loss_reenters_wait_without_stopping_provider(
        tmp_path, variant, expected):
    """Only a test-owned TF switch changes; no provider restart, command or goal is sent."""
    require_sandbox()
    root = _bundle(tmp_path)
    control = tmp_path / 'tf_control'
    control.write_text('on')
    provider = checker = None
    log, provider_log = tmp_path / 'check.log', tmp_path / 'provider.log'
    events_path = tmp_path / 'events.jsonl'
    try:
        provider = OwnedProcess([
            sys.executable, str(Path(__file__).with_name('live_check_fixture.py')),
            '--run-dir', str(root), '--tf-control', str(control)], os.environ, provider_log)
        _ready(provider, provider_log)
        executable = Path(get_package_prefix('nav_bringup')) / 'lib/nav_bringup/navcfg'
        checker = OwnedProcess([
            str(executable), 'check', '--live', '--strict', '--run-dir', str(root),
            '--sample-sec', '2', '--events', str(events_path),
            *(['--wait-timeout', '8'] if variant == 'bounded' else [])], os.environ, log)
        _wait_text(checker, log, 'localization_ready')
        control.write_text('off')
        if variant != 'bounded':
            _wait_text(checker, log, 'localization_retry')
            time.sleep(1.0)
            assert checker.poll() is None, log.read_text()
            assert provider.poll() is None, provider_log.read_text()
            if variant == 'resume':
                control.write_text('on')
            else:
                os.kill(checker.pid, signal.SIGINT)
        deadline = time.monotonic() + 25
        while checker.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        assert checker.poll() == expected, log.read_text()
        assert provider.poll() is None, 'the check must not signal the localization provider'
        events = read_events(events_path)[0]
        snapshots = [event.data['status'] for event in events
                     if event.event_type == 'interface_snapshot']
        assert snapshots == ([3, 0] if variant == 'resume' else [3])
        assert ('localization_retry' in log.read_text()) == (variant != 'bounded')
        if variant == 'resume':
            assert 'navcfg check: 0 error(s)' in log.read_text()
    finally:
        if checker is not None:
            checker.close()
        if provider is not None:
            provider.close()
