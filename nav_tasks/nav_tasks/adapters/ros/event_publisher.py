"""Owned observation publishers on the caller's node, with no extra ROS participant."""

import json
import logging
import math
import threading
import time
from typing import Callable, Mapping
import uuid

from nav_tasks.domain.tasks import DockGoal

_LOGGER = logging.getLogger(__name__)


class NavigationEvents:
    """Serialize loop/worker observations independently of motion and frozen WS events."""

    def __init__(self, *, now: Callable[[], float] = time.time) -> None:
        """Create an empty owner before allocating ROS handles.

        Args:
            now: Unix wall-clock source, independent of ROS/simulation time.
        """
        self._now = now
        self._identity = uuid.uuid4().hex
        self._sequence = 0
        self._lock = threading.Lock()
        self._node = None
        self._nav = None
        self._dock = None
        self._message = None
        self._string = None
        self._opened = False
        self._sealed = False

    def open(self, node: object, *, dock: bool = False) -> None:
        """Allocate depth-ten reliable/volatile publishers on an already-owned node.

        Retain this owner and call close even if opening the second publisher
        fails. ROS topic remapping applies normally; no extra parameters or node
        are introduced. Open/close belong to the assembly lifecycle.

        Args:
            node: Caller-owned node kept alive until close succeeds.
            dock: Also publish the separate SPEC R7 private JSON surface.

        Raises:
            RuntimeError: The owner was already opened or sealed.
            Exception: Construction failed; any completed handles remain owned.
        """
        if self._node is not None or self._sealed:
            raise RuntimeError('observation owner cannot be reopened')
        from david_nav_msgs.msg import NavEvent

        self._node, self._message = node, NavEvent
        self._nav = node.create_publisher(NavEvent, '/nav/events', 10)
        if dock:
            from std_msgs.msg import String

            self._string = String
            self._dock = node.create_publisher(String, '~/events', 10)
        self._opened = True

    def task(self, name: str, data: Mapping[str, object]) -> None:
        """Publish one manager-loop observation without affecting the task result.

        Args:
            name: Lifecycle type, separate from legacy TaskEvent kinds.
            data: Detached correlation/state payload, not arbitrary request contents.
        """
        failed = name in ('task_failed', 'task_fault', 'task_cleanup_failed')
        severity = 'error' if failed else 'info'
        self._publish('tasks', name, severity, data)

    def dock(self, goal: DockGoal, name: str, data: Mapping[str, object]) -> None:
        """Dual-write one correlated Dock measurement from its existing owned worker.

        Args:
            goal: Immutable task/generation identity, never an action to send.
            name: Workflow event, for example measured or after_spin.
            data: Unrounded workflow values in metres/radians/seconds.
        """
        if not isinstance(name, str) or not name.strip() or not isinstance(data, Mapping):
            _LOGGER.warning('Invalid Dock observation name or data')
            return
        try:
            values = dict(data, task_id=goal.request_id, sub_id=goal.sub_id,
                          generation=goal.generation)
            severity = 'warn' if name == 'no_pose' or data.get('clear') is False else 'info'
        except Exception as error:
            _LOGGER.warning('Invalid Dock observation (%s)', type(error).__name__)
            return
        self._publish('dock', 'dock_' + name, severity, values, dock_name=name)

    def close(self) -> None:
        """Seal emission and release handles after the task manager and worker settle.

        Publication and destruction share a lock. Failed destruction retains its
        handle for retry, while successfully destroyed handles are not retried.
        The caller must keep the node alive and not abandon this owner on failure.

        Raises:
            RuntimeError: ROS did not confirm a publisher was destroyed.
            Exception: Destruction failed; the remaining handles stay owned.
        """
        with self._lock:
            self._sealed = True
            for field in ('_dock', '_nav'):
                publisher = getattr(self, field)
                if publisher is not None:
                    if not self._node.destroy_publisher(publisher):
                        raise RuntimeError('failed to destroy observation publisher')
                    setattr(self, field, None)

    def _publish(self, source, name, severity, data, *, dock_name=None):
        with self._lock:
            if not self._opened or self._sealed:
                return
            try:
                if not isinstance(name, str) or not name.strip() or not isinstance(data, Mapping):
                    raise ValueError('invalid observation name or data')
                stamp = self._now()
                if type(stamp) not in (int, float) or not math.isfinite(stamp):
                    raise ValueError('invalid observation wall time')
                encoded = json.dumps(dict(data), ensure_ascii=False, allow_nan=False)
                self._sequence += 1
                event_id = f'{self._identity}:{self._sequence}'
                message = self._message(
                    event_id=event_id, stamp=float(stamp), source=source, event_type=name,
                    severity=severity, data_json=encoded)
                private = None
                if dock_name is not None and self._dock is not None:
                    private = self._string(data=json.dumps(
                        dict(data, event=dock_name, stamp=float(stamp), event_id=event_id),
                        ensure_ascii=False, allow_nan=False))
            except Exception as error:
                _LOGGER.warning('Invalid navigation observation (%s)', type(error).__name__)
                return
            for publisher, value in ((self._nav, message), (self._dock, private)):
                if publisher is not None and value is not None:
                    try:
                        publisher.publish(value)
                    except Exception as error:
                        _LOGGER.warning('Navigation observation delivery failed (%s)',
                                        type(error).__name__)
