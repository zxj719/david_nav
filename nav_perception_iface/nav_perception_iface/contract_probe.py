"""临时采样定位与底盘接口的只读探针.

持有自己的 ROS 上下文、节点和订阅，不持有接口提供方，也不创建运动发布者。
"""

from __future__ import annotations

from copy import deepcopy
import math
import time
from typing import Any, Callable, Mapping, Optional, Type

from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from nav_perception_iface.configuration import string_value
from nav_perception_iface.contracts import check_contracts, InterfaceSnapshot


class ContractProbe:
    """观察接口但不控制提供方、发布命令或初始化定位的临时探针.

    Attributes:
        profile: 有效配置的深拷贝，与调用方的后续修改分离。
        data_topics: 必需数据话题及类型，不包括只查询订阅者的底盘命令输入。
        buffer: 本对象使用的 TF 缓冲区；进入上下文前为 None。
        chassis_required: 是否检查底盘数据和命令订阅者。
        command: 只检查订阅者的底盘命令话题；不创建该话题的发布者。
        context: 本对象持有的私有 ROS 上下文；未创建时为 None。
        executor: 只驱动本对象节点的执行器；未创建时为 None。
        frames: 本次检查使用的 map、odom、body 和 nav_base 坐标系绑定。
        listener: 本对象持有的 TF 监听器；未创建时为 None。
        node: 本对象持有的只读观察节点；未创建时为 None。
    """

    def __init__(self, profile: Mapping[str, Any], chassis_required: bool = True) -> None:
        """校验契约输入和话题绑定，不初始化 ROS 或打开硬件.

        复制档案并以空快照检查所需配置；只完成本接口所需校验，不是全档案校验。
        命令和里程计话题的冲突按去除前导斜杠后的名称比较。

        Args:
            profile: 已按模式合并的机器人配置。
            chassis_required: 是否检查底盘数据和命令订阅者，必须是 bool。

        Raises:
            ValueError: 必需字段、开关或话题绑定无效，包括私有或替换模板话题名称。
        """
        if type(chassis_required) is not bool:
            raise ValueError('chassis_required must be boolean')
        self.profile = deepcopy(profile)
        profile = self.profile
        self.chassis_required = chassis_required
        check_contracts(self.profile, InterfaceSnapshot(frozenset(), {}, {}, frozenset(), {}),
                        chassis_required)
        self.frames = {name: string_value(profile, 'frames.' + name)
                       for name in ('map', 'odom', 'body', 'nav_base')}
        self.data_topics = {string_value(profile, 'localization.odom_topic'):
                            'nav_msgs/msg/Odometry'}
        self.command = string_value(profile, 'chassis.cmd_topic') if chassis_required else None
        if chassis_required:
            self.data_topics[string_value(profile, 'chassis.odom_topic')] = 'nav_msgs/msg/Odometry'
        if self.command and self.command.lstrip('/') in {
                topic.lstrip('/') for topic in self.data_topics}:
            raise ValueError('command and observed data topics must be distinct')
        if any('~' in topic or '{' in topic or '}' in topic
               for topic in (*self.data_topics, *((self.command,) if self.command else ()))):
            raise ValueError('contract topics require explicit non-private names')
        self.context = self.node = self.executor = self.listener = self.buffer = None
        self._samples = set()

    def __enter__(self) -> 'ContractProbe':
        """创建私有上下文和 TF 监听器，尚不创建里程计数据订阅.

        Returns:
            已初始化的观察器；数据订阅要等 sample 显式确认定位后才创建。

        Raises:
            BaseException: 创建失败后先尝试 close，再传播错误；
                清理自身失败可能覆盖原错误。
        """
        try:
            self.context = Context()
            rclpy.init(args=[], context=self.context,
                       signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node('nav_interface_check', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self.node, spin_thread=False)
            return self
        except BaseException:
            self.close()
            raise

    def __exit__(self, exc_type: Optional[Type[BaseException]],
                 exc_value: Optional[BaseException], traceback: Any) -> None:
        """释放本观察器资源，不向其它 ROS 进程发送信号.

        Args:
            exc_type: 当前上下文异常类型，若无异常则为 None。
            exc_value: 当前上下文异常对象。
            traceback: 当前上下文异常回溯。

        Raises:
            Exception: close 释放资源失败；可能覆盖原上下文异常。
        """
        self.close()

    def close(self) -> None:
        """按执行器、监听器、节点与上下文的顺序尝试释放已创建资源.

        执行器关闭失败会跳过同层的监听器注销，但 finally 仍尝试节点和上下文清理。
        执行器、监听器、节点的引用在各自释放调用前清空，不承诺失败后可完整重试，
        也不把重复调用无操作当作全部资源已确认释放。

        Raises:
            Exception: 资源释放失败；后续 finally 中的异常可能覆盖较早异常。
        """
        try:
            if self.executor is not None:
                executor, self.executor = self.executor, None
                executor.shutdown()
            if self.listener is not None:
                listener, self.listener = self.listener, None
                listener.unregister()
        finally:
            try:
                if self.node is not None:
                    node, self.node = self.node, None
                    node.destroy_node()
            finally:
                if self.context is not None and self.context.ok():
                    self.context.shutdown()

    def spin(self, duration_sec: float) -> None:
        """在自己的执行器上处理一次回调等待，不驱动其它节点.

        等待超时不是回调执行耗时的硬上界。

        Args:
            duration_sec: 传给 spin_once 的等待超时，单位秒；由调用方保证有效。
        """
        self.executor.spin_once(timeout_sec=duration_sec)

    def _has_transform(self, target, source, stamp=None):
        """只读查询指定时刻的 TF 可用性，同名帧按单位变换直接通过.

        不创建变换、不设置定位；缺省时刻使用最新可用变换。

        Args:
            target: 目标坐标帧。
            source: 源坐标帧，同 target 时不查询缓冲区。
            stamp: 指定 ROS 时刻；None 表示最新时刻。

        Returns:
            TF 可用性；TransformException 或 ValueError 转为 False。
        """
        if target == source:
            return True
        try:
            return self.buffer.can_transform(
                target, source, stamp if stamp is not None else Time())
        except (TransformException, ValueError):
            return False

    def localized(self) -> bool:
        """只读检查全局帧到里程计帧的 TF 是否可查询，不设置初始位姿.

        Returns:
            当前变换可用性；不另外校验新鲜度，不代表其余接口契约已经满足。
        """
        return self._has_transform(self.frames['map'], self.frames['odom'])

    def _receive(self, topic, message):
        """标记采样窗口内已收到的话题.

        Args:
            topic: 本次订阅所绑定的话题名。
            message: 消息对象；探针不保留负载。
        """
        self._samples.add(topic)

    def _snapshot(self):
        """汇总本采样窗口的消息、端点与 TF 证据，不控制定位.

        帧链查询当前可用 TF。
        速度端点仅统计 Twist 订阅者，不创建发布者或发送命令。

        Returns:
            当前接口证据快照；没有样本或变换时保留缺失，不推断定位成功。

        Raises:
            Exception: ROS 端点查询或未被变换探针处理的底层错误。
        """
        pairs = [(self.frames[target], self.frames[source]) for target, source in (
            ('map', 'odom'), ('odom', 'body'), ('body', 'nav_base'))]
        transforms = {pair for pair in pairs if self._has_transform(*pair)}
        return InterfaceSnapshot(
            frozenset(transforms),
            {topic: self.node.count_publishers(topic) for topic in self.data_topics},
            {self.command: sum(info.topic_type == 'geometry_msgs/msg/Twist' for info in
                               self.node.get_subscriptions_info_by_topic(self.command))}
            if self.command else {},
            frozenset(self._samples),
            {topic: frozenset(info.topic_type for info in
                              self.node.get_publishers_info_by_topic(topic))
             for topic in self.data_topics})

    def sample(self, duration_sec: float, stopped: Callable[[], bool]) -> InterfaceSnapshot:
        """在定位已可查询后创建临时订阅，采样并尝试释放本次订阅.

        使用 volatile、best-effort、depth-one QoS，兼容可靠和尽力投递发布者，
        不请求保留的历史样本；只记录本次收到的话题，不保存消息负载。
        这不证明消息源时间新鲜，也不排除发布方重放旧数据。

        Args:
            duration_sec: 有限正单调时钟采样窗口，单位秒。
            stopped: 调用方持有的中断检查，在创建订阅和轮询时调用。

        Returns:
            本次收到的数据、端点和 TF 证据；提前停止可返回不完整快照，由调用方判断。

        Raises:
            ValueError: 窗口无效或开始采样时定位尚未可查询。
            Exception: 订阅创建、观察或释放失败；finally 逐个释放，
                某次释放抛异常会中断后续释放，不保证每个订阅均确认销毁。
        """
        if (type(duration_sec) not in (int, float) or not math.isfinite(duration_sec)
                or duration_sec <= 0):
            raise ValueError('sample duration must be finite and positive')
        if not self.localized():
            raise ValueError('localization must be ready before data sampling')
        self._samples.clear()
        subscriptions = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        try:
            for topic, message_type in self.data_topics.items():
                if stopped():
                    break
                subscriptions.append(self.node.create_subscription(
                    Odometry, topic,
                    lambda message, key=topic: self._receive(key, message), qos))
            deadline = time.monotonic() + duration_sec
            while not stopped() and time.monotonic() < deadline:
                self.spin(max(0.0, min(0.05, deadline - time.monotonic())))
            return self._snapshot()
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
