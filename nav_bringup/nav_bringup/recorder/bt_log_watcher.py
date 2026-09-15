"""Report behavior tree escape and recovery triggers (refactor plan §10.3).

:class:`TriggerReporter` turns ``/behavior_tree_log`` status changes into an
event in the run's ``events.jsonl``, a line in the terminal and the same line
in ``logs/navrun.log``. The subscription lives in
:mod:`nav_bringup.recorder.monitor` (``navrun monitor``), which also supplies
the robot pose from TF because the tree log carries none.
"""

from __future__ import annotations

import time
from typing import Any, Callable, List, Optional, Sequence, TextIO

from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder import events

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
