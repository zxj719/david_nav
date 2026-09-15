"""Run-local recording preserves byte output without opening a ROS context."""

import io
import time

import pytest

from nav_bringup.navrun.startup_observer import StartupObserver
from nav_bringup.recorder import resources
from nav_bringup.recorder.events import Event, EventLog
from nav_bringup.recorder.run_dir import create_run_directory


def _run(tmp_path):
    return create_run_directory(tmp_path, '20260916_010000_chassis_nogit')


def test_live_forwarding_and_final_drain_preserve_all_bytes(tmp_path):
    """Live log content is visible before shutdown, with remaining bytes drained later."""
    run = _run(tmp_path)
    terminal = io.BytesIO()
    log = run.logs / 'launch.log'
    log.write_bytes('等待定位\n'.encode())
    observer = StartupObserver(run, terminal)
    observer.start()
    try:
        deadline = time.monotonic() + 5
        while not terminal.getvalue():
            assert time.monotonic() < deadline
            time.sleep(0.01)
        with log.open('ab') as stream:
            stream.write(b'x' * 180000 + b'\nfinal\n')
    finally:
        observer.close()
    assert terminal.getvalue() == log.read_bytes()
    assert (run.logs / 'console.log').read_bytes() == log.read_bytes()
    assert (run.metrics / 'resources.csv').exists()
    observer.close()


def test_short_terminal_writes_are_completed_without_dropping_bytes(tmp_path):
    """A terminal sink accepting partial writes cannot truncate console output."""
    class ShortWriter(io.BytesIO):
        def write(self, data):
            return super().write(data[:3])

    run = _run(tmp_path)
    terminal = ShortWriter()
    (run.logs / 'launch.log').write_bytes(b'partial terminal writes\n')
    observer = StartupObserver(run, terminal)
    observer.start()
    observer.close()
    assert terminal.getvalue() == b'partial terminal writes\n'


def test_broken_terminal_preserves_console_and_reports_warning(tmp_path):
    """Losing a terminal does not discard authoritative child or console logs."""
    class BrokenWriter:
        def write(self, data):
            raise BrokenPipeError('fixture disconnected terminal')

    run = _run(tmp_path)
    (run.logs / 'launch.log').write_bytes(b'saved even without terminal\n')
    observer = StartupObserver(run, BrokenWriter())
    observer.start()
    observer.close()
    assert (run.logs / 'console.log').read_bytes() == b'saved even without terminal\n'
    assert any('terminal:' in error for error in observer.errors)


def test_symlinked_logs_are_not_forwarded(tmp_path):
    """The observer does not follow run log links to unrelated files."""
    run = _run(tmp_path)
    unrelated = tmp_path / 'unrelated.txt'
    unrelated.write_bytes(b'not a run log')
    (run.logs / 'unrelated.log').symlink_to(unrelated)
    observer = StartupObserver(run, io.BytesIO())
    observer.start()
    observer.close()
    assert (run.logs / 'console.log').read_bytes() == b''


def test_console_is_never_overwritten(tmp_path):
    """Exclusive creation rejects an existing console record before starting a thread."""
    run = _run(tmp_path)
    (run.logs / 'console.log').write_bytes(b'previous')
    observer = StartupObserver(run, io.BytesIO())
    with pytest.raises(FileExistsError):
        observer.start()
    observer.close()
    assert (run.logs / 'console.log').read_bytes() == b'previous'


def test_resource_failure_does_not_hide_final_output(tmp_path, monkeypatch):
    """Resource sampling errors remain visible while log draining is still attempted."""
    def fail(self):
        raise OSError('fixture resource failure')

    monkeypatch.setattr(resources.ResourceSampler, 'sample', fail)
    run = _run(tmp_path)
    (run.logs / 'launch.log').write_bytes(b'last output')
    observer = StartupObserver(run, io.BytesIO())
    observer.start()
    observer.close()
    assert observer.errors == ['fixture resource failure']
    assert (run.logs / 'console.log').read_bytes() == b'last output'


def test_key_events_are_forwarded_once_with_partial_lines_retained(tmp_path):
    """Run events remain readable while an unfinished JSON line waits for completion."""
    run = _run(tmp_path)
    EventLog(run.events).append('navrun', 'cleanup_retry_pending', 'warn', {'pid': 123})
    second = (Event(1.0, 'navrun', 'startup_ready').to_json() + '\n').encode()
    split = len(second) // 2
    with run.events.open('ab') as stream:
        stream.write(second[:split])
    terminal = io.BytesIO()
    observer = StartupObserver(run, terminal)
    observer.start()
    try:
        deadline = time.monotonic() + 5
        while b'cleanup_retry_pending' not in terminal.getvalue():
            assert time.monotonic() < deadline
            time.sleep(0.01)
        with run.events.open('ab') as stream:
            stream.write(second[split:])
    finally:
        observer.close()
    assert terminal.getvalue().count(b'cleanup_retry_pending') == 1
    assert terminal.getvalue().count(b'startup_ready') == 1
    assert b'navrun [warn]' in terminal.getvalue()
    assert not observer.errors
