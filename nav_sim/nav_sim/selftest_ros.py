"""Private read-only selftest observer; never creates a motion publisher/client."""

import json
import math
import os
import time
from typing import Optional, Tuple

from david_nav_msgs.msg import NavEvent
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from nav_sim.model import require_sandbox
from nav_sim.selftest_scene import Scene
from nav_sim.selftest_trace import EvidenceTrace


class Observer:
    """Single-threaded private ROS context, observations and nonblocking readiness.

    Attributes:
        buffer: 本对象使用的 TF 缓冲区。
        context: 本对象持有的私有 ROS 上下文。
        error: ROS 观察失败原因；非 None 使本次验证失败。
        executor: 只驱动本对象节点的执行器；未创建时为 None。
        listener: 本对象持有的 TF 监听器；未创建时为 None。
        node: 本对象持有的只读观察节点；未创建时为 None。
    """

    def __init__(self, scene: Scene, trace: EvidenceTrace) -> None:
        """Create subscriptions before launch so early task events are observable.

        Args:
            scene: Digest-verified private graph.
            trace: Bounded exclusive writer; a failed callback invalidates the run.

        Raises:
            RuntimeError: Environment/context is not domain 99 or ROS initialization fails.
            OSError: Private profile cannot be read.
        """
        require_sandbox(os.environ)
        self.node = self.executor = self.listener = None
        self.context = Context()
        self.error = None
        self._trace, self._odom, self._map_ready = trace, None, False
        self._states, self._clients, self._pending = {}, {}, {}
        self._next_query = 0.0
        self._frames = yaml.safe_load((scene.root / 'scene/robot.yaml').read_text())['frames']
        prefix = '/' + scene.namespace + '/'
        try:
            # Read-only observer has no file logger. Avoid configuring process-wide
            # ROS logging or writing outside the exclusive experiment directory.
            self.context.init(args=[], domain_id=99, initialize_logging=False)
            if self.context.get_domain_id() != 99:
                raise RuntimeError('selftest context must use domain 99')
            self.node = Node('selftest_observer', namespace='/' + scene.namespace,
                             context=self.context, use_global_arguments=False,
                             enable_rosout=False, cli_args=[
                                 '--ros-args', '-r', '/tf:=' + prefix + 'tf',
                                 '-r', '/tf_static:=' + prefix + 'tf_static'])
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self.node)
            self.node.create_subscription(NavEvent, prefix + 'nav/events', self._event, 100)
            self.node.create_subscription(Odometry, prefix + 'odom', self._odometry, 10)
            self.node.create_subscription(
                OccupancyGrid, prefix + 'map', self._map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self._clients = {name: self.node.create_client(GetState, prefix + name + '/get_state')
                             for name in scene.graph.lifecycle_nodes}
        except BaseException:
            self.close()
            raise

    def _event(self, message):
        """接收导航事件并交给严格证据解码器，保留原始 JSON 以检测重复键.

        Args:
            message: NavEvent 消息，data_json 的 UTF-8 字节数不得超过 60000。

        解析、序列化或落盘失败时记录 error，使外层验证失败；不把坏观察
        丢弃后继续宣称成功，也不向运动接口发布内容。
        """
        try:
            # Enforce raw payload cap before decoding or serializing its envelope.
            if len(message.data_json.encode('utf-8')) > 60000:
                raise ValueError('NavEvent payload limit')
            envelope = dict(event_id=message.event_id, source=message.source,
                            event_type=message.event_type, severity=message.severity,
                            stamp=message.stamp)
            # Preserve the original JSON text for the strict duplicate-key parser.
            encoded = json.dumps(envelope, allow_nan=False)
            self._trace.receive('nav', encoded[:-1] + ',"data":' + message.data_json + '}')
        except Exception as error:
            self.error = 'nav_observation_' + type(error).__name__

    def _odometry(self, message):
        if (message.header.frame_id != self._frames['odom']
                or message.child_frame_id != self._frames['body']):
            self.error = 'unexpected_odometry_frame'
            return
        point = message.pose.pose.position
        if not all(math.isfinite(value) for value in (point.x, point.y)):
            self.error = 'invalid_odometry_position'
            return
        self._odom = (point.x, point.y, self._seconds(message.header.stamp))

    def _map(self, message):
        self._map_ready = (message.header.frame_id == self._frames['map']
                           and message.info.width > 0 and message.info.height > 0
                           and len(message.data) == message.info.width * message.info.height)

    @staticmethod
    def _seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _fresh(self, stamp):
        age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        return -0.05 <= age <= 0.5

    def step(self) -> None:
        """Pump bounded ROS work and progress each lifecycle query without blocking.

        Raises:
            RuntimeError: A callback or lifecycle service reports an observation error.
        """
        for _ in range(16):
            self.executor.spin_once(timeout_sec=0)
        if self.error:
            raise RuntimeError(self.error)
        query_now = time.monotonic() >= self._next_query
        if query_now:
            self._next_query = time.monotonic() + 0.5
        for name, client in self._clients.items():
            pending = self._pending.get(name)
            if pending is not None and pending.done():
                self._states[name] = pending.result().current_state.id
                del self._pending[name]
            if query_now and name not in self._pending and client.service_is_ready():
                self._pending[name] = client.call_async(GetState.Request())

    def pose(self) -> Optional[Tuple[float, float, float]]:
        """Return fresh map-to-navigation-centre position and yaw, never inferred success.

        Returns:
            Measured x/y/yaw, or None for unavailable/stale/invalid transform.
        """
        try:
            message = self.buffer.lookup_transform(
                self._frames['map'], self._frames['nav_base'], rclpy.time.Time())
        except TransformException:
            return None
        if not self._fresh(self._seconds(message.header.stamp)):
            return None
        point, q = message.transform.translation, message.transform.rotation
        components = (point.x, point.y, q.x, q.y, q.z, q.w)
        if (not all(math.isfinite(value) for value in components)
                or abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 1e-5):
            return None
        return (point.x, point.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def odometry(self) -> Optional[Tuple[float, float]]:
        """Return fresh fake-chassis displacement independently of TF.

        Returns:
            x/y in the configured odometry frame, or None for missing/stale data.
        """
        return self._odom[:2] if self._odom and self._fresh(self._odom[2]) else None

    def ready(self) -> bool:
        """Check map, current measurements and every lifecycle state, not just discovery.

        Returns:
            True only when every required read-only startup observation is present.
        """
        return (bool(self._clients) and set(self._states) == set(self._clients)
                and all(value == 3 for value in self._states.values())
                and self._map_ready and self.pose() is not None and self.odometry() is not None)

    def close(self) -> None:
        """Release only this observer's subscriptions, executor, node and private context."""
        try:
            if self.listener is not None:
                self.listener.unregister()
                self.listener = None
            if self.executor is not None:
                self.executor.shutdown()
                self.executor = None
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
        finally:
            self.context.try_shutdown()
