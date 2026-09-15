"""Real, private non-ROS children exercise pinned process-group ownership."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from nav_bringup.navrun.owned_process import OwnedProcess


def _start(tmp_path, code):
    return OwnedProcess([sys.executable, '-u', '-c', code], os.environ,
                        tmp_path / 'process.log')


def _until(probe, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = probe()
        if result is not None and result is not False:
            return result
        time.sleep(0.01)
    raise AssertionError('private child did not reach expected state')


def _alive(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def test_poll_preserves_exited_leader_until_close(tmp_path):
    """The zombie pins group identity until cleanup is complete."""
    process = _start(tmp_path, 'print("done"); raise SystemExit(7)')
    try:
        assert _until(process.poll) == 7
        assert process.poll() == 7
        assert Path(f'/proc/{process.pid}').exists()
        assert process.close(0.2) == 7
        assert not Path(f'/proc/{process.pid}').exists()
        assert process.close(0.2) == 7
        assert process.poll() == 7
        assert (tmp_path / 'process.log').read_text().strip() == 'done'
    finally:
        process.close(0.2)


def test_dead_leader_does_not_leave_signal_ignoring_grandchild(tmp_path):
    """An exited parent still authorizes cleanup of only its own descendants."""
    code = ('import os, signal, time\n'
            'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
            'child = os.fork()\n'
            'if child == 0:\n'
            '    while True: time.sleep(1)\n'
            'print(child, flush=True)\n'
            'os._exit(0)\n')
    process = _start(tmp_path, code)
    # A separate owned session must remain alive throughout the cleanup.
    sentinel = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                start_new_session=True)
    try:
        _until(lambda: process.poll() is not None)
        child_pid = int((tmp_path / 'process.log').read_text())
        assert _alive(child_pid)
        assert process.close(0.1) == 0
        _until(lambda: not _alive(child_pid))
        assert sentinel.poll() is None
    finally:
        process.close(0.1)
        sentinel.terminate()
        sentinel.wait(timeout=5)


def test_interrupt_waits_for_child_flush_and_preserves_exit_status(tmp_path):
    """Graceful shutdown preserves final output and the real exit code."""
    process = _start(tmp_path,
                     'import signal, time\n'
                     'def stop(*args):\n'
                     '    print("flushed", flush=True)\n'
                     '    raise SystemExit(0)\n'
                     'signal.signal(signal.SIGINT, stop)\n'
                     'print("ready", flush=True)\n'
                     'while True: time.sleep(1)\n')
    try:
        _until(lambda: 'ready' in (tmp_path / 'process.log').read_text())
        assert process.close(1) == 0
        assert 'flushed' in (tmp_path / 'process.log').read_text()
    finally:
        process.close(0.2)


def test_running_signal_ignorer_gets_bounded_kill(tmp_path):
    """Escalation only targets the private group after its grace period."""
    process = _start(tmp_path,
                     'import signal,time\n'
                     'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
                     'print("ready", flush=True)\n'
                     'while True: time.sleep(1)\n')
    try:
        _until(lambda: 'ready' in (tmp_path / 'process.log').read_text())
        assert process.close(0.1) == -signal.SIGKILL
    finally:
        process.close(0.1)


def test_no_signal_after_reap_and_existing_log_not_overwritten(tmp_path, monkeypatch):
    """Closed groups are never signaled again and logs are exclusive."""
    process = _start(tmp_path, 'raise SystemExit(0)')
    _until(lambda: process.poll() is not None)
    assert process.close(0.2) == 0

    def forbidden(*args):
        raise AssertionError('must not signal a reaped process group')

    monkeypatch.setattr(os, 'killpg', forbidden)
    assert process.close(0.2) == 0
    with pytest.raises(FileExistsError):
        _start(tmp_path, 'raise SystemExit(1)')


@pytest.mark.parametrize('timeout', [-1, float('inf'), float('nan'), True])
def test_invalid_timeout_does_not_release_ownership(tmp_path, timeout):
    """Invalid cleanup settings fail before changing child ownership."""
    process = _start(tmp_path, 'import time; time.sleep(60)')
    try:
        with pytest.raises(ValueError):
            process.close(timeout)
        assert process.poll() is None
    finally:
        process.close(0.2)


def test_spawn_error_closes_output_and_does_not_create_child(tmp_path):
    """An exec error leaves an empty diagnostic log with no open descriptor."""
    with pytest.raises(FileNotFoundError):
        OwnedProcess(['/definitely/not/an/executable'], os.environ, tmp_path / 'failed.log')
    # The file remains as evidence, but its descriptor is not leaked.
    assert (tmp_path / 'failed.log').read_bytes() == b''


def test_lost_wait_ownership_never_signals_numeric_group(tmp_path, monkeypatch):
    """An external reaper or broken child contract cannot authorize PID-only cleanup."""
    process = _start(tmp_path, 'import time; time.sleep(60)')

    def no_longer_ours(*args):
        raise ChildProcessError('fixture reaped elsewhere')

    def forbidden(*args):
        raise AssertionError('must verify wait ownership before signaling')

    try:
        with monkeypatch.context() as patch:
            patch.setattr(os, 'waitid', no_longer_ours)
            patch.setattr(os, 'killpg', forbidden)
            with pytest.raises(ChildProcessError):
                process.close(0.2)
    finally:
        process.close(0.2)


def test_unfinished_kill_retains_leader_for_safe_retry(tmp_path, monkeypatch):
    """A still-running group after escalation cannot release its reserved PID."""
    from nav_bringup.navrun import owned_process

    process = _start(tmp_path, 'import time; time.sleep(60)')
    try:
        with monkeypatch.context() as patch:
            ticks = iter(range(100))
            patch.setattr(owned_process, '_group_running', lambda _: True)
            patch.setattr(owned_process.time, 'monotonic', lambda: next(ticks))
            patch.setattr(owned_process.time, 'sleep', lambda _: None)
            with pytest.raises(subprocess.TimeoutExpired):
                process.close(0)
        assert Path(f'/proc/{process.pid}').exists()
        assert process.close(0.2) in (-signal.SIGINT, -signal.SIGKILL)
    finally:
        process.close(0.2)


def test_only_explicit_descriptors_survive_exec(tmp_path):
    """A keyboard can inherit capabilities without leaking unrelated parent files."""
    read_fd, write_fd = os.pipe()
    process = None
    try:
        code = ('import os,sys\n'
                'os.write(int(sys.argv[1]), b"ready")\n'
                'try: os.fstat(int(sys.argv[2]))\n'
                'except OSError: raise SystemExit(0)\n'
                'raise SystemExit(7)\n')
        process = OwnedProcess(
            [sys.executable, '-c', code, str(write_fd), str(read_fd)], os.environ,
            tmp_path / 'fds.log', pass_fds=(write_fd,))
        _until(lambda: process.poll() is not None)
        assert process.poll() == 0
        assert os.read(read_fd, 5) == b'ready'
        assert not os.get_inheritable(write_fd)
    finally:
        if process is not None:
            process.close(0.2)
        os.close(read_fd)
        os.close(write_fd)


@pytest.mark.parametrize('fds', [(True,), (-1,), (0,), ('3',)])
def test_invalid_passed_descriptors_fail_before_log_creation(tmp_path, fds):
    """Malformed handoff arguments cannot start a child or create an output log."""
    with pytest.raises(ValueError):
        OwnedProcess([sys.executable, '-c', 'pass'], os.environ,
                     tmp_path / 'fds.log', pass_fds=fds)
    assert not (tmp_path / 'fds.log').exists()
