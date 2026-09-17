"""带启动保护的键盘运行层，区分独立锁与显式委托的资源所有权.

保护检查不是实体安全确认；本模块不启动或停止定位栈，不拥有真实底盘驱动。
"""

import argparse
from dataclasses import asdict
import math
import os
import signal
import sys
import threading
import time
from typing import Any, Optional, Sequence

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from chassis_teleop.control import KeyboardState, TeleopConfig
from chassis_teleop.interlock import CommandConflict, wait_for_clear_topics
from chassis_teleop.terminal import TerminalKeyboard, TerminalUnavailable
from nav_bringup.navrun.locking import LockBusy, RunLock
from nav_bringup.navrun.locking import DEFAULT_LOCK_PATH
from nav_bringup.navrun.delegated_lock import DelegatedLock
from nav_bringup.navrun.keyboard_channel import KeyboardChannel, KeyboardOwnerLost


class KeyboardNode(Node):
    """先声明并校验参数，在显式 arm 前不创建命令发布者.

    Attributes:
        cmd_publisher: 仅在显式 arm 后创建的 Twist 发布者；此前为 None。
        config: 启动参数解析出的遥控配置。
        discovery_sec: 发布者发现窗口，单位秒。
        guard_topics: 启动前检查发布者冲突的已解析命令话题。
        state: 纯按键意图及失活超时状态。
    """

    def __init__(self, **kwargs: Any) -> None:
        """加载启动参数，不发布速度也不读取终端.

        输出话题会加入冲突检查列表并去重；这里只构造状态，尚未做图发现。

        Args:
            **kwargs: 传给 ROS Node 的标准关键字参数。

        Raises:
            ValueError: 键盘参数、冲突话题列表或发现时长不合法。
            Exception: ROS 节点、参数或话题名解析失败；不承诺部分构造回滚。
        """
        super().__init__('chassis_teleop', **kwargs)
        defaults = asdict(TeleopConfig())
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = TeleopConfig(**{name: self.get_parameter(name).value for name in defaults})
        self.state = KeyboardState(self.config)
        self.declare_parameter('guard_topics', ['/cmd_vel', '/cmd_vel_nav'])
        self.declare_parameter('discovery_sec', 1.0)
        topics = self.get_parameter('guard_topics').value
        if (not isinstance(topics, (list, tuple)) or not topics
                or not all(isinstance(topic, str) and topic for topic in topics)):
            raise ValueError('guard_topics must be a nonempty command-topic list')
        self.guard_topics = tuple(dict.fromkeys(
            self.resolve_topic_name(topic) for topic in [*topics, self.config.cmd_vel_topic]))
        self.discovery_sec = self.get_parameter('discovery_sec').value
        if (isinstance(self.discovery_sec, bool)
                or not isinstance(self.discovery_sec, (int, float))
                or not math.isfinite(self.discovery_sec) or self.discovery_sec <= 0.0):
            raise ValueError('discovery_sec must be finite and positive')
        self.cmd_publisher = None

    def arm(self) -> None:
        """创建命令输出，要求调用方事先完成锁、终端与图发现检查.

        本方法不重复验证这些外部前置条件，也不发布第一条命令。

        Raises:
            RuntimeError: 已经创建过命令发布者。
            Exception: ROS 发布者创建失败。
        """
        if self.cmd_publisher is not None:
            raise RuntimeError('keyboard output already armed')
        self.cmd_publisher = self.create_publisher(Twist, self.config.cmd_vel_topic, 10)

    def handle_key(self, key: str) -> str:
        """处理一个完整按键事件，停车键立即触发一次零速尝试.

        Args:
            key: 完整终端字符或转义序列；运动时间取当前单调时钟。

        Returns:
            motion、stop、quit 或 ignored，供前台循环处理；
            quit 本身不在此发布退出零速。
        """
        event = self.state.handle_key(key, time.monotonic())
        if event == 'stop':
            self.publish_zero()
        return event

    def publish_current_command(self) -> None:
        """在已启用输出后发布带按键失活保护的当前命令.

        只设置 Twist 的前向与偏航分量，其它分量为零；发送成功不代表硬件执行。

        Raises:
            RuntimeError: 尚未创建命令发布者。
            Exception: 取样或 ROS 发布失败。
        """
        if self.cmd_publisher is None:
            raise RuntimeError('keyboard output is not armed')
        message = Twist()
        message.linear.x, message.angular.z = self.state.command(time.monotonic())
        self.cmd_publisher.publish(message)

    def publish_zero(self) -> None:
        """仅在已持有发布者时尝试一次零速，未启用的输出保持无发布者.

        发布 Exception 转为警告，不等待硬件停车确认；日志设施自身错误不在此捕获。
        """
        if self.cmd_publisher is not None:
            try:
                self.cmd_publisher.publish(Twist())
            except Exception as error:
                self.get_logger().warning(f'Unable to publish keyboard stop: {error}')

    def publish_stop_burst(self) -> None:
        """已启用输出时沿用五次零速退出尝试，不触碰未启用的命令图.

        每次尝试后均阻塞 0.02 秒，包括最后一次；不据此认定实体已经停稳。
        """
        if self.cmd_publisher is not None:
            for _ in range(5):
                self.publish_zero()
                time.sleep(0.02)


def run_keyboard(
    node: KeyboardNode, keyboard: TerminalKeyboard, stopped: threading.Event,
    channel: Optional[KeyboardChannel] = None,
) -> None:
    """在调用方持有导航租约时执行启动保护和键盘前台循环.

    先检查发布者冲突，委托模式再通知 ready 并最多等待 30 秒的启用许可。
    轮询前后检查监督者所有权；函数自身不负责最终零速、关终端或销毁节点，
    这些由外层生命周期处理。

    Args:
        node: 调用方持有且尚未 arm 的键盘节点。
        keyboard: 调用方已打开的终端读取器。
        stopped: INT/TERM 停止标记，信号处理器本身不发布或销毁 ROS 对象。
        channel: 可选的监督者通道，与其继承租约共同校验。

    Raises:
        KeyboardOwnerLost: 委托启用失败、等待超时或监督者所有权丢失。
        CommandConflict: 启动发布者检查失败。
        TerminalUnavailable: 终端读取失败或断开。
        KeyboardInterrupt: 启动等待期间收到停止请求或 ROS 已关闭。
        Exception: ROS、时钟或其它调用失败，交给外层清理。
    """
    def spin(duration: float) -> None:
        if stopped.is_set() or not rclpy.ok():
            raise KeyboardInterrupt
        rclpy.spin_once(node, timeout_sec=duration)

    wait_for_clear_topics(node.guard_topics, node.get_publishers_info_by_topic,
                          spin, time.monotonic, node.discovery_sec)
    if stopped.is_set() or not rclpy.ok():
        return
    if channel is not None:
        channel.ready()
        deadline = time.monotonic() + 30.0
        while not channel.permitted():
            if time.monotonic() >= deadline:
                raise KeyboardOwnerLost('supervisor activation timed out')
            spin(0.05)
        if stopped.is_set() or not rclpy.ok():
            return
    node.arm()
    print('Keyboard ready: WASD/arrows (or configured uiojkl); space/k stop; q/Ctrl-C quit.',
          flush=True)
    while rclpy.ok() and not stopped.is_set():
        if channel is not None:
            channel.check_owner()
        key = keyboard.read_key(node.config.period_sec)
        if stopped.is_set():
            break
        if channel is not None:
            channel.check_owner()
        if key and node.handle_key(key) == 'quit':
            break
        node.publish_current_command()
        rclpy.spin_once(node, timeout_sec=0.0)


def main(args: Optional[Sequence[str]] = None) -> int:
    """持有独占导航锁运行独立键盘入口，并检查启动发布者冲突.

    正常启动在 arm 前完成锁、终端和图发现检查；运行中的同类异常也会映射为
    返回码 2，因此仅凭返回 2 不能推断整个进程从未创建过发布者。

    Args:
        args: 可选 ROS 参数；None 使用进程命令行。

    Returns:
        正常退出、信号或已处理的 ROS 关闭返回零；
        锁、终端、图发现、配置或操作系统错误打印后返回 2。

    Raises:
        Exception: 不属于上述已处理类别的运行或清理错误。
    """
    try:
        with RunLock():
            return _run(args)
    except (LockBusy, CommandConflict, TerminalUnavailable, ValueError, OSError) as error:
        print(f'chassis_teleop: {error}', file=sys.stderr)
        return 2


def managed_main(args: Optional[Sequence[str]] = None) -> int:
    """使用继承的锁租约、监督者通道与终端描述符运行委托键盘.

    这不是基于环境变量绕过锁：锁描述符必须携带 DEFAULT_LOCK_PATH 的既有
    独占 flock，仅打开路径不足以通过检查。参数解析及三个不同非标准描述符的
    初检在接管前完成；通过初检进入 try 后，finally 尝试关闭三个传入描述符。
    监督者仍应保留自己的租约，并在键盘进程结束后恢复终端。

    Args:
        args: 显式内部描述符参数及后续 ROS 参数；None 使用命令行。

    Returns:
        正常退出或信号返回零；捕获的委托、终端、图发现、配置、系统错误或
        所有权丢失打印后返回 2。

    Raises:
        SystemExit: 参数解析处理帮助或拒绝非法的描述符参数。
        Exception: 未被此入口捕获的运行或清理错误。
    """
    parser = argparse.ArgumentParser(description='Supervisor-owned keyboard entry')
    for name in ('lock', 'control', 'tty'):
        parser.add_argument(f'--{name}-fd', type=int, required=True)
    selected, ros_args = parser.parse_known_args(args)
    descriptors = (selected.lock_fd, selected.control_fd, selected.tty_fd)
    if any(fd < 3 for fd in descriptors) or len(set(descriptors)) != 3:
        parser.error('three distinct nonstandard descriptors are required')
    try:
        for descriptor in descriptors:
            os.set_inheritable(descriptor, False)
        with DelegatedLock(selected.lock_fd, DEFAULT_LOCK_PATH):
            with KeyboardChannel(selected.control_fd) as channel:
                return _run(ros_args, channel, selected.tty_fd)
    except (CommandConflict, TerminalUnavailable, ValueError, OSError,
            KeyboardOwnerLost) as error:
        print(f'chassis_teleop: {error}', file=sys.stderr)
        return 2
    finally:
        for descriptor in descriptors:
            try:
                os.close(descriptor)
            except OSError:
                pass


def _run(
    args: Optional[Sequence[str]], channel: Optional[KeyboardChannel] = None,
    tty_fd: Optional[int] = None,
) -> int:
    """持有键盘节点的 ROS 与信号生命周期，清理成功创建的自有资源.

    ROS 初始化位于 try/finally 之外，失败直接传播。初始化成功后才安装临时
    信号处理器并构造节点；终端上下文成功进入后，键盘循环终止会先尝试零速。
    节点变量尚未赋值时不做节点级销毁，最终仍尝试关闭 ROS 并恢复已记录的处理器。

    Args:
        args: 传给 rclpy 的 ROS 参数；None 使用进程参数。
        channel: 可选监督者通道，调用方持有并负责最终关闭。
        tty_fd: 可选已授权终端描述符；键盘对象只复制并关闭自己的副本。

    Returns:
        正常结束、键盘中断或 ROS 外部关闭时返回零。

    Raises:
        Exception: 未处理的初始化、键盘运行、零速尝试或清理错误；
            清理错误也会传播，不承诺实体停车。
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous = {}
    stopped = threading.Event()
    node = None

    def stop(_signum: int, _frame: Any) -> None:
        stopped.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)
        node = KeyboardNode()
        terminal = (TerminalKeyboard(node.config.tty_device) if tty_fd is None else
                    TerminalKeyboard(node.config.tty_device, descriptor=tty_fd))
        with terminal as keyboard:
            try:
                run_keyboard(node, keyboard, stopped, channel)
            finally:
                node.publish_stop_burst()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
    return 0
