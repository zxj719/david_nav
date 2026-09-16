"""Drive cooperative CLI shutdown through the real supervisor with owned non-ROS children."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from nav_bringup.navrun import cli
from nav_bringup.navrun.locking import RunLock
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest


@pytest.mark.parametrize('phase', ['host', 'gate', 'running'])
def test_down_finalizes_real_record_after_owned_children_drain(tmp_path, capsys, phase):
    """Cooperative requests traverse actual stop probes, cleanup, finalization and unlock."""
    waiting = 'import time; print("waiting fixture", flush=True); time.sleep(60)'
    code = (
        'import sys, time\n'
        f'sys.path.insert(0, {str(Path(__file__).parent)!r})\n'
        'from pathlib import Path\n'
        'from dataclasses import replace\n'
        'from test_recorded_startup import _options, _commands, _run, StartupGate, startup\n'
        f'root = Path({str(tmp_path)!r})\n'
        'def pause(plan, events, stopped):\n'
        '    (root / "host_ready").write_text("ready")\n'
        '    while stopped() is None: time.sleep(0.01)\n'
        'def commands(plan):\n'
        '    selected = _commands(plan)\n'
        f'    if {phase!r} == "gate":\n'
        f'        gate = StartupGate("ready", (sys.executable, "-u", "-c", {waiting!r}))\n'
        '        selected = replace(selected, gates=(gate,))\n'
        '    return selected\n'
        f'steps = (startup.StartupStep("pause", pause),) if {phase!r} == "host" else ()\n'
        'result, output = _run(_options(root), commands=commands, steps=steps)\n'
        'raise SystemExit(result.exit_code)\n')
    process = subprocess.Popen(
        [sys.executable, '-c', code], start_new_session=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=dict(os.environ, ROS_DOMAIN_ID='99'))
    run = tmp_path / 'runs/20260916_010000_chassis_nogit'
    deadline = time.monotonic() + 15
    try:
        while True:
            if phase == 'host':
                ready = (tmp_path / 'host_ready').exists()
            elif phase == 'gate':
                log = run / 'logs/ready.log'
                ready = log.is_file() and b'waiting fixture' in log.read_bytes()
            else:
                ready = (run / 'events.jsonl').exists() and any(
                    item.event_type == 'startup_ready'
                    for item in read_events(run / 'events.jsonl')[0])
            if ready:
                break
            if process.poll() is not None:
                pytest.fail(process.stdout.read().decode(errors='replace'))
            assert time.monotonic() < deadline, 'owned fixture did not become ready'
            time.sleep(0.02)
        path = tmp_path / 'lease'
        assert cli.main(['status', '--lock-path', str(path)]) == 0
        active = json.loads(capsys.readouterr().out)
        assert active['state'] == 'active' and active['pid'] == process.pid
        assert cli.main(['down', '--lock-path', str(path), '--wait-timeout', '15']) == 0
        result = json.loads(capsys.readouterr().out)
        assert result['state'] == 'finished' and result['cleanup_complete'] is True
        assert process.wait(timeout=10) == 143
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == 143
        assert (run / 'summary.md').is_file()
        for event in read_events(run / 'events.jsonl')[0]:
            if event.event_type == 'process_started':
                assert not Path('/proc', str(event.data['pid'])).exists()
        with RunLock(path):
            pass
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=20)
        process.stdout.close()
