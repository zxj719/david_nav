"""Sandbox integration checks for the recorder ROS side (PROGRESS P1.2b, P1.4a).

These checks start ROS processes, so ``colcon test`` must not collect them:
the file name deliberately does not match ``test_*.py``. Run them explicitly
in the isolated DDS domain while no navigation stack runs::

    nav_bringup/sandbox_tests/run_sandbox_checks.sh

They start a topic publisher, a static transform, a fake behavior tree log,
``navrun monitor`` and ``navrun dump-params``. Every process runs in its own
session, and only those sessions are stopped afterwards.
"""

import csv
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import IO, List, Optional, Sequence, Tuple

import pytest
import yaml

from nav_bringup.recorder import events
from nav_bringup.recorder import param_dump
from nav_bringup.recorder import topic_stats

LIVE_PATTERN = (
    'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|'
    'navigation_executor|dock_navigator|rviz2')
CHATTER_TOPIC = '/navrun_sandbox/chatter'
SANDBOX_BASE_FRAME = 'navrun_sandbox_base'


class _Sessions:
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


def _require_sandbox() -> None:
    expected = os.environ.get('SANDBOX_ROS_DOMAIN_ID', '99')
    if os.environ.get('ROS_DOMAIN_ID') != expected:
        pytest.fail(f'sandbox checks need ROS_DOMAIN_ID={expected}')
    live = subprocess.run(
        ['pgrep', '-f', LIVE_PATTERN], capture_output=True, text=True, check=False)
    if live.stdout.strip():
        pytest.fail('a navigation, localization or chassis stack is running')


@pytest.fixture
def sessions(tmp_path):
    """Provide process sessions that are stopped after the check."""
    _require_sandbox()
    log_dir = tmp_path / 'process_logs'
    log_dir.mkdir()
    started = _Sessions(log_dir)
    yield started
    started.stop_all()


def _static_transform(child_frame: str) -> List[str]:
    return ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--frame-id', 'map', '--child-frame-id', child_frame]


def test_monitor_records_topics_tf_and_triggers(sessions, tmp_path):
    """The monitor writes topic rates, TF lag and a behavior tree trigger."""
    run_path = tmp_path / '20260915_000000_sandbox_nogit'
    sessions.start('chatter', [
        'ros2', 'topic', 'pub', '-r', '10', CHATTER_TOPIC, 'std_msgs/msg/String',
        '{data: sandbox}'])
    sessions.start('static_tf', _static_transform(SANDBOX_BASE_FRAME))
    monitor = sessions.start('monitor', [
        'ros2', 'run', 'nav_bringup', 'navrun', 'monitor', '--run-dir', str(run_path),
        '--base-frame', SANDBOX_BASE_FRAME, '--topic', CHATTER_TOPIC, '--period', '1'])
    time.sleep(5.0)
    sessions.start('bt_log', [
        'ros2', 'topic', 'pub', '-r', '2', '/behavior_tree_log',
        'nav2_msgs/msg/BehaviorTreeLog',
        '{event_log: [{node_name: RobotEmbeddedInObstacle, previous_status: IDLE, '
        'current_status: SUCCESS}]}'])
    time.sleep(8.0)

    assert sessions.stop(monitor) is not None

    csv_path = run_path / 'metrics' / topic_stats.TOPICS_CSV_NAME
    with csv_path.open(encoding='utf-8', newline='') as handle:
        rows = list(csv.DictReader(handle))
    chatter_rates = [float(row['rate_hz']) for row in rows if row['name'] == CHATTER_TOPIC]
    summary = topic_stats.summarize_topics(csv_path)
    chatter = {stats.name: stats for stats in summary.topics}[CHATTER_TOPIC]
    transform = {stats.name: stats for stats in summary.transforms}[
        f'map->{SANDBOX_BASE_FRAME}']
    assert max(chatter_rates) >= 5.0
    assert chatter.publishers_max >= 1
    assert transform.lag_max_ms is not None

    recorded, _ = events.read_events(run_path / 'events.jsonl')
    triggers = [event for event in recorded if event.event_type == 'escape_triggered']
    assert triggers
    assert triggers[0].data['node'] == 'RobotEmbeddedInObstacle'
    assert '脱困触发' in (run_path / 'logs' / 'navrun.log').read_text(encoding='utf-8')


def test_dump_params_dumps_a_live_node(sessions, tmp_path):
    """dump-params writes the parameters of a running node."""
    sessions.start('static_tf', _static_transform(SANDBOX_BASE_FRAME))
    time.sleep(3.0)
    output = tmp_path / 'dump'

    completed = subprocess.run(
        ['ros2', 'run', 'nav_bringup', 'navrun', 'dump-params',
         '--output-dir', str(output), '--spin-time', '3'],
        capture_output=True, text=True, timeout=600, check=False)

    report = yaml.safe_load((output / param_dump.REPORT_FILE_NAME).read_text(encoding='utf-8'))
    publishers = [node for node in report['dumped'] if 'static_transform_publisher' in node]
    assert publishers, completed.stderr
    dump = (output / param_dump.node_file_name(publishers[0])).read_text(encoding='utf-8')
    assert 'ros__parameters' in dump
