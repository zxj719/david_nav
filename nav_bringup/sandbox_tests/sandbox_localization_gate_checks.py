"""Prove localization waiting precedes launch and readiness clocks in real recorded up."""

import os
from pathlib import Path
import signal
import sys
import time

from conftest import require_sandbox
from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest


def test_manual_localization_wait_precedes_launch_and_cleans_its_observer(tmp_path):
    """A real TF wait outlives short readiness budgets without launching any provider."""
    require_sandbox()
    root = tmp_path / 'private_run'
    log = tmp_path / 'wrapper.log'
    process = OwnedProcess([
        sys.executable, str(Path(__file__).with_name('localization_gate_fixture.py')),
        '--root', str(root)], os.environ, log)
    try:
        deadline = time.monotonic() + 20
        run = None
        while process.poll() is None and time.monotonic() < deadline:
            link = root / 'runs/latest'
            if link.exists():
                run = link.resolve()
                events = read_events(run / 'events.jsonl')[0]
                if any(event.event_type == 'localization_waiting' for event in events):
                    break
            time.sleep(0.05)
        assert run is not None and process.poll() is None, log.read_text()
        events = read_events(run / 'events.jsonl')[0]
        assert any(event.event_type == 'localization_waiting' for event in events), log.read_text()
        time.sleep(1.0)  # Exceeds both post-launch readiness/check deadlines in the fixture.
        assert process.poll() is None, log.read_text()
        os.kill(process.pid, signal.SIGINT)
        deadline = time.monotonic() + 20
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        assert process.poll() == 130, log.read_text()
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == 130
        events = read_events(run / 'events.jsonl')[0]
        started = [event for event in events if event.event_type == 'process_started']
        assert [event.data['name'] for event in started] == ['localization']
        assert not any(event.event_type == 'stop_zero_sent' for event in events)
        assert all(not Path('/proc', str(event.data['pid'])).exists() for event in started)
        with RunLock(root / 'lease'):
            pass
    finally:
        process.close()
