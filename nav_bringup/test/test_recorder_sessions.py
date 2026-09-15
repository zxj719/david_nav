"""Tests for stopping process sessions."""

import signal
import subprocess
import sys
import time

from nav_bringup.recorder import sessions


def _start(code):
    return subprocess.Popen(
        [sys.executable, '-c', code], start_new_session=True, stdout=subprocess.PIPE)


def test_stop_session_interrupts_a_process():
    """A process that handles SIGINT exits with its own status."""
    process = _start(
        'import signal, sys, time\n'
        'signal.signal(signal.SIGINT, lambda *args: sys.exit(0))\n'
        'print("ready", flush=True)\n'
        'time.sleep(30)\n')
    assert process.stdout.readline() == b'ready\n'

    assert sessions.stop_session(process, timeout_sec=10.0) == 0
    process.stdout.close()


def test_stop_session_kills_a_process_that_ignores_sigint():
    """A process that ignores SIGINT is killed when the timeout expires."""
    process = _start(
        'import signal, time\n'
        'signal.signal(signal.SIGINT, signal.SIG_IGN)\n'
        'print("ready", flush=True)\n'
        'time.sleep(30)\n')
    assert process.stdout.readline() == b'ready\n'
    started = time.monotonic()

    assert sessions.stop_session(process, timeout_sec=0.5) == -signal.SIGKILL
    assert time.monotonic() - started < 10.0
    process.stdout.close()


def test_stop_session_of_a_finished_process():
    """A process that already exited reports its status."""
    process = _start('pass')
    process.wait(timeout=10)

    assert sessions.stop_session(process, timeout_sec=1.0) == 0
    process.stdout.close()
