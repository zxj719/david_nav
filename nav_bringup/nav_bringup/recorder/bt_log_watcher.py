"""Report behavior tree escape and recovery triggers of a running navigation.

``navrun watch-bt`` subscribes to Nav2's ``/behavior_tree_log``, feeds the
status changes to :class:`~nav_bringup.recorder.bt_triggers.TriggerDetector`
and, for every trigger, appends an event to the run's ``events.jsonl``, prints
a line to the terminal and appends the same line to ``logs/navrun.log``
(refactor plan §10.3). The tree log carries no pose, so the pose is looked up
in TF (``map`` → ``base_footprint`` by default).

ROS packages are imported only inside :func:`spin`, so the rest of
``nav_bringup`` and its unit tests do not need a ROS environment.
"""

from __future__ import annotations

import sys
import time
from typing import Any, Callable, List, Optional, Sequence, TextIO

from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder import events
from nav_bringup.recorder import run_dir

EVENT_SOURCE = 'bt_watch'
LOG_FILE_NAME = 'navrun.log'


def status_changes_from_msg(msg: Any) -> List[bt_triggers.StatusChange]:
    """Convert a ``nav2_msgs/BehaviorTreeLog`` message.

    Args:
        msg: The message, or any object with the same fields.

    Returns:
        The status changes in message order.
    """
    return [
        bt_triggers.StatusChange(
            node_name=str(entry.node_name),
            previous_status=str(entry.previous_status),
            current_status=str(entry.current_status),
            stamp=entry.timestamp.sec + entry.timestamp.nanosec * 1e-9,
        )
        for entry in msg.event_log
    ]


class TriggerReporter:
    """Write the triggers of status changes to the event log and text sinks."""

    def __init__(
        self,
        detector: bt_triggers.TriggerDetector,
        event_log: Optional[events.EventLog],
        sinks: Sequence[TextIO],
        pose_provider: Callable[[], Optional[bt_triggers.Pose2D]] = lambda: None,
        task_provider: Callable[[], Optional[str]] = lambda: None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        """Create a reporter.

        Args:
            detector: Detector that decides what is reported.
            event_log: Run event log, or ``None`` to only print lines.
            sinks: Text streams that receive one line per trigger, for example
                the terminal and ``logs/navrun.log``.
            pose_provider: Returns the current robot pose, or ``None``.
            task_provider: Returns the active task id, or ``None``.
            clock: Wall-clock time source for the printed lines.
        """
        self._detector = detector
        self._event_log = event_log
        self._sinks = list(sinks)
        self._pose_provider = pose_provider
        self._task_provider = task_provider
        self._clock = clock

    def handle(self, changes: Sequence[bt_triggers.StatusChange]) -> List[bt_triggers.Trigger]:
        """Process status changes and report the triggers they fire.

        Args:
            changes: Status changes, typically one message's worth.

        Returns:
            The reported triggers.
        """
        triggers = self._detector.process(changes)
        for trigger in triggers:
            pose = self._pose_provider()
            task_id = self._task_provider()
            if self._event_log is not None:
                self._event_log.append(
                    EVENT_SOURCE, trigger.rule.event_type, trigger.rule.severity,
                    trigger.event_data(pose, task_id))
            line = bt_triggers.format_trigger_line(trigger, self._clock(), pose, task_id)
            for sink in self._sinks:
                sink.write(line + '\n')
                sink.flush()
        return triggers


def spin(
    run: Optional[run_dir.RunDirectory] = None,
    *,
    topic: str = '/behavior_tree_log',
    map_frame: str = 'map',
    base_frame: str = 'base_footprint',
    min_interval_sec: float = 0.5,
    stdout: TextIO = sys.stdout,
) -> int:
    """Report triggers until interrupted or ROS shuts down.

    Args:
        run: Run directory to record into, or ``None`` to only print.
        topic: Behavior tree log topic.
        map_frame: Frame of the reported pose.
        base_frame: Robot frame whose pose is reported.
        min_interval_sec: Minimum tree time between reports of one rule.
        stdout: Terminal stream.

    Returns:
        Exit status 0.
    """
    from nav2_msgs.msg import BehaviorTreeLog
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.time import Time
    import tf2_ros

    rclpy.init()
    node = rclpy.create_node('navrun_bt_watch')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    def lookup_pose() -> Optional[bt_triggers.Pose2D]:
        """Look up the latest robot pose in TF.

        Returns:
            The pose, or ``None`` while the transform is unavailable.
        """
        try:
            transform = tf_buffer.lookup_transform(map_frame, base_frame, Time())
        except tf2_ros.TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = bt_triggers.yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return bt_triggers.Pose2D(translation.x, translation.y, yaw, map_frame)

    sinks: List[TextIO] = [stdout]
    event_log = None
    log_file = None
    if run is not None:
        run.logs.mkdir(parents=True, exist_ok=True)
        log_file = (run.logs / LOG_FILE_NAME).open('a', encoding='utf-8')
        sinks.append(log_file)
        event_log = events.EventLog(run.events)
    reporter = TriggerReporter(
        bt_triggers.TriggerDetector(min_interval_sec=min_interval_sec), event_log, sinks,
        pose_provider=lookup_pose)
    node.create_subscription(
        BehaviorTreeLog, topic, lambda msg: reporter.handle(status_changes_from_msg(msg)), 10)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.try_shutdown()
        if log_file is not None:
            log_file.close()
    return 0
