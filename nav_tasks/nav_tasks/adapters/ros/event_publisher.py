"""在外部节点持有导航观察发布器，不增加 ROS 节点或执行线程."""

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
    """串行化循环及工作线程观察，与运动控制和冻结 WS 事件分离."""

    def __init__(self, *, now: Callable[[], float] = time.time) -> None:
        """创建空观察所有者、生产者身份与序号，不分配 ROS 句柄.

        Args:
            now: Unix 墙钟秒数来源，独立于 ROS 或仿真时钟，发布时验证有限性。
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
        """在现有节点创建可靠、易失、深度十的观察发布器.

        创建第二个发布器失败时，仍须保留本对象并调用 close。
        只在完整打开后允许发布；不新增节点或参数，正常应用 ROS 名称重映射。
        open/close 由外层按生命周期顺序调用，不允许关闭后重新打开。

        Args:
            node: 外部持有的节点，存活到 close 成功。
            dock: 同时创建 SPEC R7 的节点私有 JSON 观察出口。

        Raises:
            RuntimeError: 已持有节点或已封闭。
            Exception: 导入或创建失败，已创建句柄仍由本对象持有。
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
        """同步发布任务循环的观察，不改变任务控制结果或冻结事件格式.

        失败、故障及清理失败标为 error，其余为 info；编码/发布普通异常
        在下层记录后丢弃，不持久化、不重试或等待 DDS 确认。日志本身仍可能失败。

        Args:
            name: 生命周期名称，与旧 TaskEvent 种类分离。
            data: 调用方提供的关联/状态映射，不应包含任意请求正文。
        """
        failed = name in ('task_failed', 'task_fault', 'task_cleanup_failed')
        severity = 'error' if failed else 'info'
        self._publish('tasks', name, severity, data)

    def dock(self, goal: DockGoal, name: str, data: Mapping[str, object]) -> None:
        """把带任务身份的 Dock 测量写入全局观察及可选私有 JSON 出口.

        复制顶层数据并覆盖任务/子任务/代次字段；no_pose 或 clear 为 False
        使用 warn。两次发布独立失败，共享事件 ID 和墙钟时间，不形成交付事务。

        Args:
            goal: 不可变任务与代次身份，不会被当作动作下发。
            name: 工作流事件名，例如 measured 或 after_spin。
            data: 未舍入的观测数据，距离/角度/时间单位为米、弧度、秒。
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
        """封闭事件发布，并在任务管理器和 Dock 工作线程收尾后销毁自有句柄.

        发布与销毁共享锁。先私有、后全局，首个失败中断后续销毁；成功项清除，
        失败项及尚未尝试项保留供重试。外部节点必须继续存活，不在这里销毁。

        Raises:
            RuntimeError: 节点以假值表示发布器未被销毁。
            Exception: 销毁抛错，封闭状态与剩余句柄保持。
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
        """在发布与销毁共享锁内构造观察事件，坏数据或投递失败仅记警告.

        导航事件和可选 Dock 镜像共享事件 ID 与秒制时间戳；发布失败不回滚
        序号、不重试、不影响运动控制。未打开或已封闭时直接忽略。

        Args:
            source: 观察来源名称。
            name: 非空导航事件名称。
            severity: 事件严重程度标签。
            data: 可序列化为有限数值 JSON 的观察数据映射。
            dock_name: Dock 镜像的工作流事件名；None 表示不构造镜像。
        """
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
