"""Read TF, graph discovery and GetParameters through one disposable ROS context."""

from __future__ import annotations

from collections import Counter
import math
import time
from typing import Any, Callable, Dict, Optional, Type

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from nav_bringup.navcfg.live_check import LiveSnapshot, LiveSpec


class RosProbe:
    """Own a read-only ROS observer, with no action client or command publisher."""

    def __init__(self, spec: LiveSpec) -> None:
        """Describe the observation without initializing ROS.

        Args:
            spec: Expected frames, topics and parameters.
        """
        self.spec = spec
        self.context = None
        self.node = None
        self.listener = None
        self.buffer = None
        self.executor = None

    def __enter__(self) -> 'RosProbe':
        """Create one private context and TF listener, cleaning up partial failures.

        Returns:
            This initialized observer.
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
        """Release only this observer's resources.

        Args:
            exc_type: Active exception type, if any.
            exc_value: Active exception, if any.
            traceback: Active traceback, if any.
        """
        self.close()

    def close(self) -> None:
        """Destroy this listener/node/context without touching another ROS participant."""
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
        """Process observer callbacks for at most one bounded poll.

        Args:
            duration_sec: Maximum wall-clock poll duration.
        """
        self.executor.spin_once(timeout_sec=duration_sec)

    def localized(self) -> bool:
        """Check whether the configured global-to-odometry transform exists.

        Returns:
            True only for an available transform (or identical configured frames).
        """
        return self.spec.map_frame == self.spec.odom_frame or self.buffer.can_transform(
            self.spec.map_frame, self.spec.odom_frame, Time())

    def _age(self) -> Optional[float]:
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
        return dict(Counter('/'.join((namespace.rstrip('/'), name))
                            for name, namespace in self.node.get_node_names_and_namespaces()))

    def snapshot(self, stopped: Callable[[], bool], timeout_sec: float = 5.0) -> LiveSnapshot:
        """Query all selected parameter services concurrently within one global deadline.

        Args:
            stopped: Caller-owned interrupt flag.
            timeout_sec: Global parameter-discovery/response budget, not per-node timeout.

        Returns:
            Fresh graph/TF facts and successful parameter responses. Failed, missing
            and undeclared parameters remain absent so the assessor reports them.

        Raises:
            ValueError: The time budget is not finite and positive.
        """
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('parameter timeout must be finite and positive')
        counts = self._counts()
        clients, pending, parameters = {}, {}, {}
        deadline = time.monotonic() + timeout_sec
        try:
            for name in self.spec.parameters:
                if counts.get(name, 0) == 1:
                    clients[name] = self.node.create_client(
                        GetParameters, name + '/get_parameters')
            remaining = set(clients)
            while remaining and not stopped() and time.monotonic() < deadline:
                for name in tuple(remaining):
                    try:
                        if name not in pending and clients[name].service_is_ready():
                            request = GetParameters.Request(names=list(self.spec.parameters[name]))
                            pending[name] = clients[name].call_async(request)
                        if name in pending and pending[name].done():
                            values = pending[name].result().values
                            names = list(self.spec.parameters[name])
                            if len(names) == len(values):
                                converted = [parameter_value_to_python(value) for value in values]
                                parameters[name] = {
                                    key: value for key, value in zip(names, converted)
                                    if value is not None}
                            remaining.remove(name)
                    except Exception:
                        remaining.remove(name)
                if remaining:
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
