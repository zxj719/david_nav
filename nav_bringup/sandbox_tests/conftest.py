"""Shared fixtures of the nav_bringup sandbox integration checks.

The checks run only through ``run_sandbox_checks.sh``. The ``sessions``
fixture refuses to run outside the sandbox DDS domain or while a navigation
stack runs, starts every process in its own session, and stops only those
sessions afterwards.
"""

import os
from pathlib import Path
import signal
import subprocess
from typing import IO, List, Optional, Sequence, Set, Tuple

import pytest

LIVE_PATTERN = (
    'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|'
    'navigation_executor|dock_navigator|rviz2')


class ProcessSessions:
    """Processes started in their own sessions, stopped in reverse order."""

    def __init__(self, log_dir: Path) -> None:
        """Create an empty set of sessions.

        Args:
            log_dir: Directory for one log file per process.
        """
        self._log_dir = log_dir
        self._started: List[Tuple[subprocess.Popen, IO[str]]] = []

    def start(self, name: str, argv: Sequence[str]) -> subprocess.Popen:
        """Start a process as the leader of a new session.

        Args:
            name: Log file name without extension.
            argv: Program and arguments.

        Returns:
            The started process.
        """
        log = (self._log_dir / f'{name}.log').open('w', encoding='utf-8')
        process = subprocess.Popen(
            list(argv), stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self._started.append((process, log))
        return process

    def stop(self, process: subprocess.Popen, timeout_sec: float = 15.0) -> Optional[int]:
        """Interrupt the session of a process, killing it if it does not exit.

        Args:
            process: A process returned by :meth:`start`.
            timeout_sec: Seconds to wait after the interrupt.

        Returns:
            The exit status of the session leader.
        """
        self._signal_session(process, signal.SIGINT)
        try:
            return process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            self._signal_session(process, signal.SIGKILL)
            return process.wait()

    def stop_all(self) -> None:
        """Stop every started session, newest first, and close the logs."""
        for process, log in reversed(self._started):
            self.stop(process)
            self._signal_session(process, signal.SIGKILL)
            log.close()

    @staticmethod
    def _signal_session(process: subprocess.Popen, signum: int) -> None:
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            pass


def _ancestor_pids() -> Set[int]:
    pids: Set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in pids:
        pids.add(pid)
        try:
            with open(f'/proc/{pid}/stat', encoding='utf-8') as handle:
                stat = handle.read()
        except OSError:
            break
        pid = int(stat[stat.rindex(')') + 2:].split()[1])
    return pids


def require_sandbox() -> None:
    """Fail unless the sandbox domain is set and no navigation stack runs.

    ``pgrep -f`` matches whole command lines, so the processes that started
    these checks are ignored: their command lines may name watched processes.
    """
    expected = os.environ.get('SANDBOX_ROS_DOMAIN_ID', '99')
    if os.environ.get('ROS_DOMAIN_ID') != expected:
        pytest.fail(f'sandbox checks need ROS_DOMAIN_ID={expected}')
    live = subprocess.run(
        ['pgrep', '-f', LIVE_PATTERN], capture_output=True, text=True, check=False)
    ancestors = _ancestor_pids()
    running = [pid for pid in live.stdout.split() if int(pid) not in ancestors]
    if running:
        pids = ', '.join(running)
        pytest.fail(f'a navigation, localization or chassis stack is running (pid {pids})')


@pytest.fixture
def sessions(tmp_path):
    """Provide process sessions that are stopped after the check."""
    require_sandbox()
    log_dir = tmp_path / 'process_logs'
    log_dir.mkdir()
    started = ProcessSessions(log_dir)
    yield started
    started.stop_all()
