"""Disposable read-only sampling of localization, chassis and optional cloud contracts."""

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
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from nav_perception_iface.configuration import string_value
from nav_perception_iface.contracts import check_contracts, InterfaceSnapshot


class ContractProbe:
    """Observe interfaces without owning their providers or publishing commands.

    Attributes:
        profile: A private snapshot of the effective configuration.
        data_topics: Required message topics, excluding the chassis command input.
        buffer: 本对象使用的 TF 缓冲区。
        chassis_required: 是否检查底盘数据和命令订阅者。
        cloud: 可选的点云输入话题；关闭点云检查时为 None。
        command: 只检查订阅者的底盘命令话题；不创建该话题的发布者。
        context: 本对象持有的私有 ROS 上下文。
        executor: 只驱动本对象节点的执行器；未创建时为 None。
        frames: 本次检查使用的 map/odom/body/nav_base 坐标系绑定。
        listener: 本对象持有的 TF 监听器；未创建时为 None。
        node: 本对象持有的只读观察节点；未创建时为 None。
        scan_target: 点云投影目标坐标系；未启用点云检查时为 None。
    """

    def __init__(self, profile: Mapping[str, Any], chassis_required: bool = True) -> None:
        """Validate the entire contract without initializing ROS or opening hardware.

        Args:
            profile: Mode-merged robot configuration.
            chassis_required: Whether to check chassis data and command subscribers.

        Raises:
            ValueError: A required configuration field or topic binding is invalid.
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
        scan = profile['sensors']['scan_from_cloud']
        self.cloud = string_value(profile, 'sensors.scan_from_cloud.cloud_topic') if (
            scan.get('enabled', False)) else None
        self.scan_target = string_value(profile, 'frames.scan_target') if self.cloud else None
        if self.cloud:
            if self.cloud.lstrip('/') in {topic.lstrip('/') for topic in self.data_topics}:
                raise ValueError('cloud and odometry topics must be distinct')
            self.data_topics[self.cloud] = 'sensor_msgs/msg/PointCloud2'
        if self.command and self.command.lstrip('/') in {
                topic.lstrip('/') for topic in self.data_topics}:
            raise ValueError('command and observed data topics must be distinct')
        if any('~' in topic or '{' in topic or '}' in topic
               for topic in (*self.data_topics, *((self.command,) if self.command else ()))):
            raise ValueError('contract topics require explicit non-private names')
        self.context = self.node = self.executor = self.listener = self.buffer = None
        self._samples = set()
        self._cloud_frame = self._cloud_stamp = None

    def __enter__(self) -> 'ContractProbe':
        """Create a private context and TF listener, but no data subscriptions yet.

        Returns:
            This initialized observer; data sampling starts only after localization.
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
        """Release this observer without signaling another ROS process.

        Args:
            exc_type: Active exception type, if any.
            exc_value: Active exception, if any.
            traceback: Active traceback, if any.
        """
        self.close()

    def close(self) -> None:
        """Destroy the private executor, listener, node and context, including partial setup."""
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
        """Process callbacks for at most the supplied bounded wall-clock interval.

        Args:
            duration_sec: Maximum poll duration.
        """
        self.executor.spin_once(timeout_sec=duration_sec)

    def _has_transform(self, target, source, stamp=None):
        if target == source:
            return True
        try:
            return self.buffer.can_transform(
                target, source, stamp if stamp is not None else Time())
        except (TransformException, ValueError):
            return False

    def localized(self) -> bool:
        """Check global-to-odometry TF without requiring or initializing a pose.

        Returns:
            Whether the user-owned localization transform is presently queryable.
        """
        return self._has_transform(self.frames['map'], self.frames['odom'])

    def _receive(self, topic, message):
        self._samples.add(topic)
        if topic == self.cloud:
            self._cloud_frame = message.header.frame_id
            try:
                self._cloud_stamp = Time.from_msg(message.header.stamp)
            except ValueError:
                self._cloud_stamp = None

    def _snapshot(self):
        """汇总本采样窗口的消息、端点与 TF 证据，不记录点云负载或控制定位.

        常规帧链查询当前可用 TF；点云转换必须使用最后收到样本的时间戳。
        速度端点仅统计 Twist 订阅者，不创建发布者或发送命令。

        Returns:
            当前接口证据快照；没有样本或变换时保留缺失，不推断定位成功。

        Raises:
            Exception: ROS 端点查询或未被变换探针处理的底层错误。
        """
        pairs = [(self.frames[target], self.frames[source]) for target, source in (
            ('map', 'odom'), ('odom', 'body'), ('body', 'nav_base'))]
        transforms = {pair for pair in pairs if self._has_transform(*pair)}
        if self._cloud_frame and self._cloud_stamp is not None and self._has_transform(
                self.scan_target, self._cloud_frame, self._cloud_stamp):
            transforms.add((self.scan_target, self._cloud_frame))
        return InterfaceSnapshot(
            frozenset(transforms),
            {topic: self.node.count_publishers(topic) for topic in self.data_topics},
            {self.command: sum(info.topic_type == 'geometry_msgs/msg/Twist' for info in
                               self.node.get_subscriptions_info_by_topic(self.command))}
            if self.command else {},
            frozenset(self._samples),
            {topic: frozenset(info.topic_type for info in
                              self.node.get_publishers_info_by_topic(topic))
             for topic in self.data_topics}, self._cloud_frame)

    def sample(self, duration_sec: float, stopped: Callable[[], bool]) -> InterfaceSnapshot:
        """Collect fresh messages only after localization, releasing every subscription.

        Subscribers use volatile best-effort depth-one QoS: reliable and best-effort
        publishers are supported, but retained pre-localization samples cannot pass.
        Only topic presence and the latest cloud header are kept, not payload data.

        Args:
            duration_sec: Finite positive wall-clock sampling window.
            stopped: Caller-owned interruption predicate, checked at each bounded poll.

        Returns:
            Actual data, endpoint and TF evidence from this window. Cloud TF is
            queried at the latest received cloud's stamp, not an unrelated latest TF.

        Raises:
            ValueError: The sampling window is invalid or localization is not ready.
        """
        if (type(duration_sec) not in (int, float) or not math.isfinite(duration_sec)
                or duration_sec <= 0):
            raise ValueError('sample duration must be finite and positive')
        if not self.localized():
            raise ValueError('localization must be ready before data sampling')
        self._samples.clear()
        self._cloud_frame = self._cloud_stamp = None
        subscriptions = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        try:
            for topic, message_type in self.data_topics.items():
                if stopped():
                    break
                message_class = PointCloud2 if message_type == 'sensor_msgs/msg/PointCloud2' else (
                    Odometry)
                subscriptions.append(self.node.create_subscription(
                    message_class, topic,
                    lambda message, key=topic: self._receive(key, message), qos))
            deadline = time.monotonic() + duration_sec
            while not stopped() and time.monotonic() < deadline:
                self.spin(max(0.0, min(0.05, deadline - time.monotonic())))
            return self._snapshot()
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)
