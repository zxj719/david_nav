"""提供私有只读自测观察器，不创建运动命令发布器或目标客户端."""

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
    """持有私有 ROS 上下文，以单线程轮询处理观察及生命周期查询.

    Attributes:
        buffer: 本对象使用的 TF 缓冲区，初始化到相应步骤后才存在。
        context: 本对象持有的私有 ROS 上下文。
        error: ROS 观察失败原因；非 None 使本次验证失败。
        executor: 只驱动本对象节点的执行器；未创建时为 None。
        listener: 本对象持有的 TF 监听器；未创建时为 None。
        node: 本对象持有的只读观察节点；未创建时为 None。
    """

    def __init__(self, scene: Scene, trace: EvidenceTrace) -> None:
        """在被观察图启动前创建订阅，尽早接收事件但不保证 DDS 永不丢失.

        初始化 ROS 的 try 块内失败会调用 close；档案读取在该块之前。
        close 的异常可能覆盖原始初始化异常。

        Args:
            scene: 已通过摘要核验的私有场景。
            trace: 有界排他记录器；回调失败会使本次验证失败。

        Raises:
            RuntimeError: 环境/实际域不是 99，或 ROS 初始化失败。
            OSError: 私有档案无法读取。
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
            self._events_topic = prefix + 'nav/events'
            self.node.create_subscription(NavEvent, self._events_topic, self._event, 100)
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
        """检查自测里程计帧与二维位置后保存位置和消息时间.

        帧或位置不合格时记录持久错误，不清除以前的样本；新鲜度另行检查。
        仅观察导航自测数据，不控制或修改定位栈。

        Args:
            message: 自测订阅收到的里程计消息，含 header 和 child_frame_id。
        """
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
        """根据帧名、正尺寸与数据长度更新自测地图就绪标志.

        只证明基本结构满足启动观察条件，不证明地图占用值、分辨率或目标可达。

        Args:
            message: 当前收到的静态地图消息，后续无效帧会使就绪标志变为 False。
        """
        self._map_ready = (message.header.frame_id == self._frames['map']
                           and message.info.width > 0 and message.info.height > 0
                           and len(message.data) == message.info.width * message.info.height)

    @staticmethod
    def _seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _fresh(self, stamp):
        """用节点 ROS 时钟判断样本时间是否处于允许的新鲜度窗口.

        允许最多 0.05 秒的未来偏差和 0.5 秒的历史年龄；不使用单调墙钟。
        这只是消息新鲜度判断，不是停稳或运动完成判断。

        Args:
            stamp: 与节点 ROS 时钟同口径的样本时间，单位秒。

        Returns:
            当前时间减样本时间位于闭区间 [-0.05, 0.5] 时返回 True。
        """
        age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        return -0.05 <= age <= 0.5

    def step(self) -> None:
        """轮询最多 16 次 ROS 工作并推进生命周期异步查询.

        spin_once 使用零等待；执行次数受限不意味着回调自身具有墙钟时限。
        查询每隔至少 0.5 秒发起，已有未完成请求不重复发送。

        Raises:
            RuntimeError: 已记录观察错误；回调及服务 Future 的其它异常原样传播。
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
        """读取新鲜的地图到导航旋转中心位姿，不从任务成功消息推算.

        Returns:
            x/y/yaw 三元组，单位米/弧度。TF 不可用、时间不新鲜、
            所检查分量非有限或四元数模平方偏差超过 1e-5 时为 None。
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
        """独立于 TF 返回新鲜假底盘二维位置，位移由调用方计算.

        Returns:
            配置里程计帧下的 x/y，单位米；缺失或时间不新鲜时为 None。
        """
        return self._odom[:2] if self._odom and self._fresh(self._odom[2]) else None

    def ready(self) -> bool:
        """综合地图结构、当前测量和缓存生命周期状态判断启动就绪.

        事件订阅是易失的：发布者尚未被发现时发出的事件收不到。就绪因此要求本节点已经发现
        至少一个事件发布者，否则协调器可能在订阅配上对之前就下发第一个任务，导致首条
        任务边界事件丢失、裁判误判（2026-09-20 实跑到）。

        Returns:
            存在生命周期客户端且其缓存状态全为 active(3)，地图基本结构有效，
            TF 及里程计新鲜、且已发现事件发布者时为 True。状态缓存没有独立年龄检查，
            结果不是目标可达或运动完成证明。
        """
        return (bool(self._clients) and set(self._states) == set(self._clients)
                and all(value == 3 for value in self._states.values())
                and self._map_ready and self.pose() is not None
                and self.odometry() is not None
                and self.node.count_publishers(self._events_topic) > 0)

    def close(self) -> None:
        """按顺序尝试释放本观察器的监听器、执行器、节点和私有上下文.

        前三项任一步骤抛异常会跳过后续同层步骤，但 finally 仍尝试关闭上下文。
        关闭上下文本身抛异常可覆盖先前异常，不保证每种失败路径都完整释放。
        """
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
