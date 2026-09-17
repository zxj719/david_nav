"""为纯速度适配器提供 ROS 通信、状态诊断和有序退出时的零速尝试.

节点不持有串口或定位栈；发布命令本身不是实体执行或停稳的证据。
"""

from dataclasses import asdict
import signal
import threading
import time
from typing import Any, Optional, Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from chassis_adapter.core import AdapterConfig, AdapterState


class AdapterNode(Node):
    """适配配置指定的速度话题，不拥有硬件驱动或定位栈.

    Attributes:
        cmd_publisher: 配置输出话题的 Twist 发布者。
        config: 启动参数解析出的适配器配置。
        diagnostic_publisher: 状态诊断发布者，不代表实体已执行命令。
        state: 纯速度整形与超时状态。
    """

    def __init__(self, **kwargs: Any) -> None:
        """声明兼容旧入口的参数，校验后创建发布、订阅和诊断定时器.

        Args:
            **kwargs: 传给 rclpy Node 的标准选项，包括参数覆盖值。

        Raises:
            ValueError: 适配器配置校验失败。
            Exception: 节点、参数或通信端点创建失败；本构造函数不承诺部分创建回滚。
        """
        super().__init__('chassis_adapter', **kwargs)
        defaults = asdict(AdapterConfig())
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = AdapterConfig(**{
            name: self.get_parameter(name).value for name in defaults})
        self.config.validate()
        self.state = AdapterState(self.config, self._now())
        self.cmd_publisher = self.create_publisher(Twist, self.config.output_cmd_vel_topic, 10)
        self.create_subscription(
            Twist, self.config.input_cmd_vel_topic, self.velocity_callback, 10)
        self.create_timer(self.config.period_sec, self.publish_current_command)
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._diagnostic_time = time.monotonic()
        self._diagnostic_count = 0
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'Command adapter ready: {self.config.input_cmd_vel_topic} -> '
            f'{self.config.output_cmd_vel_topic}; timeout={self.config.cmd_vel_timeout:.2f}s')

    def _now(self) -> int:
        return self.get_clock().now().nanoseconds

    def velocity_callback(self, message: Twist) -> None:
        """记录整形命令，由定时器发布，不在订阅回调内发布.

        Args:
            message: 输入 Twist；输出只保留前向和偏航，原始横向值仍参与
                角速度最小幅值选择。接收时间取当前节点 ROS 时钟。
        """
        self.state.receive(message.linear.x, message.linear.y, message.angular.z, self._now())

    def publish_current_command(self) -> None:
        """发布最新命令，输入过期后只尝试一次零速，再保持静默.

        纯状态返回 None 时不发布；消息未赋值的其它轴保持零。
        发布失败不回滚已经消费的单次超时标记，不在此确认硬件停车。
        """
        output = self.state.output(self._now())
        if output is not None:
            message = Twist()
            message.linear.x, message.angular.z = output
            self.cmd_publisher.publish(message)

    def publish_diagnostics(self) -> None:
        """在一赫兹定时器中发布输入速率、过期标记和整形速度诊断.

        输入速率按单调时钟统计窗口计算，过期判断仍用 ROS 时间。
        这是一份软件状态快照，不是底盘的速度反馈或命令执行确认。
        """
        now = time.monotonic()
        elapsed = now - self._diagnostic_time
        values = self.state.diagnostics(self._now())
        values['input_hz'] = ((self.state.input_count - self._diagnostic_count) / elapsed
                              if elapsed > 0.0 else 0.0)
        self._diagnostic_time = now
        self._diagnostic_count = self.state.input_count
        status = DiagnosticStatus()
        status.name = self.get_fully_qualified_name()
        status.level = DiagnosticStatus.WARN if values['timed_out'] else DiagnosticStatus.OK
        status.message = 'input stale: output released after stop' if values['timed_out'] else 'OK'
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def publish_zero(self) -> None:
        """尝试发布一次全零 Twist，发布 Exception 转为警告.

        不等待底盘反馈或停车确认；日志设施自身失败不在此隔离。
        """
        try:
            self.cmd_publisher.publish(Twist())
        except Exception as error:
            # Report transport failure; publishing is not evidence of a hardware stop.
            self.get_logger().warning(f'Unable to publish stop command: {error}')

    def publish_stop_burst(self, repetitions: int = 5, period_sec: float = 0.02) -> None:
        """在销毁发布者前执行多次零速尝试，默认沿用五次发送.

        每次尝试后都会等待，包括最后一次；不是仅在两次发送之间等待。

        Args:
            repetitions: 尝试次数，先转为 int 且至少为一，默认五。
            period_sec: 每次尝试后的阻塞间隔，单位秒，先转为 float 并下限截到零。
        """
        for _ in range(max(int(repetitions), 1)):
            self.publish_zero()
            time.sleep(max(float(period_sec), 0.0))


def main(args: Optional[Sequence[str]] = None) -> int:
    """运行适配器，收到 INT 或 TERM 后尝试零速并释放节点和 ROS 上下文.

    ROS 初始化成功后，主体结束走 finally 清理并恢复信号处理器；
    上下文已关闭或发布失败时，零速尝试不能保证实体停车。
    初始化位于清理边界之外，失败直接向调用方传播。

    Args:
        args: 可选 ROS 命令行参数；None 使用运行时默认参数来源。

    Returns:
        有序退出或已处理的键盘中断、外部 ROS 关闭返回零。

    Raises:
        Exception: 未处理的运行或清理错误；主体中的异常会先经过清理路径，
            清理本身失败也会传播。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopped = threading.Event()
    previous = {}
    node = None

    def stop(_signum: int, _frame: Any) -> None:
        stopped.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)
        node = AdapterNode()
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.publish_stop_burst()
                finally:
                    node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
    return 0
