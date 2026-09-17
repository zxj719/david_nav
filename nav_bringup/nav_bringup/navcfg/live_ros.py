"""用一个用完即弃的私有 ROS 上下文读取 TF、图发现与参数服务.

全程只读：不建发布者、不建动作客户端、不写参数，也不安装信号处理器。所有资源都属于
本对象，关闭时只销毁自己的监听器、节点与上下文，不影响同机其它 ROS 参与者。
"""

from __future__ import annotations

from collections import Counter
import math
import time
from typing import Any, Callable, Dict, Optional, Type

from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from nav_bringup.navcfg.live_check import LiveSnapshot, LiveSpec


class RosProbe:
    """持有一个只读 ROS 观察者，没有动作客户端，也没有速度发布者.

    Attributes:
        buffer: 本对象使用的 TF 缓冲区。
        context: 本对象持有的私有 ROS 上下文。
        executor: 只驱动本对象节点的执行器；未创建时为 None。
        listener: 本对象持有的 TF 监听器；未创建时为 None。
        node: 本对象持有的只读观察节点；未创建时为 None。
        spec: 预期坐标系、话题和参数的只读检查配置。
    """

    def __init__(self, spec: LiveSpec) -> None:
        """只描述这次观察，不初始化 ROS.

        Args:
            spec: 期望的坐标系、话题与参数。
        """
        self.spec = spec
        self.context = None
        self.node = None
        self.listener = None
        self.buffer = None
        self.executor = None

    def __enter__(self) -> 'RosProbe':
        """建立一个私有上下文与 TF 监听器，部分失败时自行清理.

        用独立的 Context 并关闭信号处理，既不接管调用方的信号，也不影响同机其它参与者。

        Returns:
            已初始化的本观察者。
        """
        try:
            self.context = Context()
            rclpy.init(args=[], context=self.context,
                       signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node('navcfg_live_check', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self.node, spin_thread=False)
        except BaseException:
            self.close()
            raise
        return self

    def __exit__(self, exc_type: Optional[Type[BaseException]],
                 exc_value: Optional[BaseException], traceback: Any) -> None:
        """只释放本观察者自己的资源.

        Args:
            exc_type: 当前异常类型，没有则为 None。
            exc_value: 当前异常对象，没有则为 None。
            traceback: 当前异常栈，没有则为 None。
        """
        self.close()

    def close(self) -> None:
        """销毁本对象的监听器、节点与上下文，不碰其它 ROS 参与者.

        按执行器、监听器、节点、上下文的顺序释放，前一步失败也继续走完后面几步；
        只关闭本对象持有的上下文，不做全局 rclpy 关闭。
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
        """处理一次有界的观察者回调，必须在上下文内调用.

        Args:
            duration_sec: 本次轮询最多等待的墙钟秒数。
        """
        self.executor.spin_once(timeout_sec=duration_sec)

    def localized(self) -> bool:
        """判断配置的全局到里程计变换是否存在.

        Returns:
            变换可用返回 True；两个坐标系配置成同一个时直接返回 True，不查 TF。
        """
        return self.spec.map_frame == self.spec.odom_frame or self.buffer.can_transform(
            self.spec.map_frame, self.spec.odom_frame, Time())

    def _age(self) -> Optional[float]:
        """读取最新地图到导航基准 TF 的秒制年龄，不裁剪未来时间戳.

        Returns:
            节点当前 ROS 时间减 TF 时间；相同帧为零，变换不可用为 None。
            新鲜度阈值由上层判定，本函数不发布初始位姿。
        """
        if self.spec.map_frame == self.spec.nav_frame:
            return 0.0
        try:
            transform = self.buffer.lookup_transform(
                self.spec.map_frame, self.spec.nav_frame, Time())
        except TransformException:
            return None
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        return (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9

    def _counts(self) -> Dict[str, int]:
        """按完整节点名计数，保留重复实例而不把节点列表转成集合.

        命名空间去掉尾斜杠再拼接名称，用于发现同名节点；不判断节点所在主机。

        Returns:
            完整节点名到当前发现数量的映射。

        Raises:
            Exception: ROS 图节点查询失败。
        """
        return dict(Counter('/'.join((namespace.rstrip('/'), name))
                            for name, namespace in self.node.get_node_names_and_namespaces()))

    def snapshot(self, stopped: Callable[[], bool], timeout_sec: float = 5.0) -> LiveSnapshot:
        """在一个全局期限内完成图/声明发现与选定参数的读取.

        C++ 的参数服务会因为批次里有一个未声明的键而整批拒绝，所以先用 ListParameters
        取回已声明的键，再只读这些键的值：未声明的键保持缺失，其余值照常拿到。两个服务
        都只读，不写参数。

        期限是整体预算，不是每个节点各有一份：节点迟迟未被发现、参数服务不响应、导航 TF
        还没出现，都在同一个预算里消耗。单个节点查询过程中抛出的异常会被吞掉，该节点的
        参数保持缺失，由评估方报成发现，这里不区分“查询失败”与“从未答复”。返回的快照也
        不是同一瞬间的一致视图：TF、发布者数与节点列表是在循环结束后分别采集的。

        Args:
            stopped: 调用方持有的中断标志。
            timeout_sec: 参数发现/响应的全局预算，不是每节点超时。

        Returns:
            新鲜的图/TF 事实与成功取回的参数。失败、缺失、未声明以及取值未设定的参数
            一律保持缺失，交由评估方报出。

        Raises:
            ValueError: 时间预算不是有限正数。
        """
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('parameter timeout must be finite and positive')
        clients, pending, parameters, selected = {}, {}, {}, {}
        deadline = time.monotonic() + timeout_sec
        try:
            # DDS participant/node discovery can lag action-server discovery.
            # Missing nodes consume the same global budget, not an immediate failure.
            remaining = set(self.spec.parameters)
            while (not stopped() and time.monotonic() < deadline
                   and (remaining or self._age() is None)):
                counts = self._counts()
                for name in tuple(remaining):
                    if counts.get(name, 0) != 1:
                        continue
                    reading = name in selected
                    key = (name, reading)
                    if key not in clients:
                        service, suffix = ((GetParameters, '/get_parameters') if reading
                                           else (ListParameters, '/list_parameters'))
                        clients[key] = self.node.create_client(service, name + suffix)
                    try:
                        if name not in pending and clients[key].service_is_ready():
                            request = (GetParameters.Request(names=selected[name]) if reading
                                       else ListParameters.Request(
                                           prefixes=[], depth=0))
                            pending[name] = clients[key].call_async(request)
                        if name in pending and pending[name].done():
                            response = pending.pop(name).result()
                            if not reading:
                                declared = set(response.result.names)
                                selected[name] = [key for key in self.spec.parameters[name]
                                                  if key in declared]
                                if not selected[name]:
                                    parameters[name] = {}
                                    remaining.remove(name)
                                continue
                            values = response.values
                            names = selected[name]
                            if len(names) == len(values):
                                converted = [parameter_value_to_python(value) for value in values]
                                parameters[name] = {
                                    key: value for key, value in zip(names, converted)
                                    if value is not None}
                            remaining.remove(name)
                    except Exception:
                        remaining.remove(name)
                if remaining or self._age() is None:
                    self.spin(max(0.0, min(0.05, deadline - time.monotonic())))
        finally:
            for future in pending.values():
                if not future.done():
                    future.cancel()
            for client in clients.values():
                self.node.destroy_client(client)
        return LiveSnapshot(self.localized(), self._age(),
                            {rule.topic: self.node.count_publishers(rule.topic)
                             for rule in self.spec.topics}, parameters, self._counts(),
                            {name: tuple(types) for name, types
                             in self.node.get_topic_names_and_types()})
