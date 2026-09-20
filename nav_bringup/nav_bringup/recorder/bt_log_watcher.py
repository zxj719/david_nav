"""报告行为树的脱困与恢复触发（方案 §10.3）.

:class:`TriggerReporter` 把 ``/behavior_tree_log`` 的状态变化转成运行目录
``events.jsonl`` 里的一条事件、终端上的一行，以及 ``logs/navrun.log`` 里同样的一行。
订阅本身在 :mod:`nav_bringup.recorder.monitor`（``navrun monitor``）里，
机器人位姿也由它从 TF 取来——行为树日志本身不带位姿。
本模块只做观察与记录：不取消目标、不改变导航行为。
"""

from __future__ import annotations

import time
from typing import Any, Callable, List, Optional, Sequence, TextIO

from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder import events

EVENT_SOURCE = 'bt_watch'
LOG_FILE_NAME = 'navrun.log'


def status_changes_from_msg(msg: Any) -> List[bt_triggers.StatusChange]:
    """转换一条 ``nav2_msgs/BehaviorTreeLog`` 消息.

    只做字段搬运，不判重、不过滤，也不要求消息来自真实 ROS。

    Args:
        msg: 该消息，或任何具有相同字段的对象。

    Returns:
        按消息内顺序排列的状态变化。
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
    """把状态变化触发的结果写进事件日志与文本输出.

    不是线程安全的：订阅方需保证串行调用。任一输出端异常都会向上抛出，
    不会被吞掉当成"已记录"。
    """

    def __init__(
        self,
        detector: bt_triggers.TriggerDetector,
        event_log: Optional[events.EventLog],
        sinks: Sequence[TextIO],
        pose_provider: Callable[[], Optional[bt_triggers.Pose2D]] = lambda: None,
        task_provider: Callable[[], Optional[str]] = lambda: None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        """创建一个触发报告器.

        构造阶段不订阅任何话题、不打开任何文件。

        Args:
            detector: 决定什么该被报告的检测器。
            event_log: 运行事件日志；为 ``None`` 时只打印文本行。
            sinks: 每个触发各收到一行文本的输出流，例如终端与 ``logs/navrun.log``。
            pose_provider: 返回当前机器人位姿，取不到时返回 ``None``。
            task_provider: 返回当前任务 ID，没有时返回 ``None``。
            clock: 打印行所用的墙钟时间源。
        """
        self._detector = detector
        self._event_log = event_log
        self._sinks = list(sinks)
        self._pose_provider = pose_provider
        self._task_provider = task_provider
        self._clock = clock

    def handle(self, changes: Sequence[bt_triggers.StatusChange]) -> List[bt_triggers.Trigger]:
        """处理状态变化并报告它们触发的项.

        位姿或任务 ID 取不到时照常报告，只是相应字段缺省，
        因此事件里没有位姿不代表当时机器人没在动。

        Args:
            changes: 状态变化，通常是一条消息的量。

        Returns:
            已报告的触发项。
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
