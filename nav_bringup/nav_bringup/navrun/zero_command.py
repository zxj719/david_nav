"""提供导航所有者调用的仅零速退出辅助器，不证明实体已停稳.

没有速度值参数；外层必须先取得导航所有权，拒绝启动时不得向他人图发布。
发现等待有预算，但 ROS 初始化、spin 和关闭不构成总硬期限；监督器另控子进程。
导入本模块不初始化 ROS，命令行帮助保持原文兼容。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from typing import Optional, Sequence


_CLI_DESCRIPTION = """Bounded zero-only shutdown helper invoked by the owning startup supervisor.

There is deliberately no velocity argument. Publishing a zero Twist is an
attempt to stop, not evidence that a physical robot stopped. The supervisor
must own navigation before invoking this helper; rejected startup must not
publish into somebody else's graph. Imports do not initialize ROS.
"""


class _RosZeros:
    """持有短生命周期零速发布资源，不提供通用可重试销毁协议.

    仅由调用线程驱动 SingleThreadedExecutor；不另建回调线程。

    Attributes:
        topics: 外层已验证并去重的话题序列。
        context: 自建 ROS context，进入上下文前为 None。
        node: 自建节点，创建前为 None。
        executor: 自建单线程 executor，创建前为 None。
        publishers: 已创建的零速发布器，随节点销毁，不逐项清空。
        message_type: 进入上下文时导入的 Twist 类型，之前尚未设置。
    """

    def __init__(self, topics):
        """保存话题并置空资源字段，不导入或初始化 ROS.

        Args:
            topics: 调用方已检查的绝对话题序列，不在此重复验证。
        """
        self.topics = topics
        self.context = None
        self.node = None
        self.executor = None
        self.publishers = []

    def __enter__(self):
        """创建独立 context、节点和单线程 executor，再逐话题创建发布器.

        禁用 rclpy 信号处理器，不创建背景 spin 线程。context 创建后发生
        BaseException 时尝试 __exit__；回滚异常可能替代原构造异常。

        Returns:
            当前零速传输对象。

        Raises:
            BaseException: 导入、创建或部分构造后的回滚失败。
        """
        from geometry_msgs.msg import Twist
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        self.message_type = Twist
        self.context = Context()
        try:
            rclpy.init(args=[], context=self.context,
                       signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node('navrun_stop_zero', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            for topic in self.topics:
                self.publishers.append(self.node.create_publisher(Twist, topic, 10))
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *args):
        """按 executor、节点、context 顺序尽力收尾，嵌套 finally 不跳过后续阶段.

        shutdown 不带期限，其返回值未检查；异常也继续尝试销毁节点和
        仍有效的 context。后续异常可能覆盖前一异常，不清空句柄、不保证
        幂等重试，也不能作为任意宿主 executor 已静止的通用证明。

        Args:
            args: 上下文退出异常信息，当前实现不读取或抑制它。

        Raises:
            BaseException: 资源关闭失败，后续 finally 仍会尝试执行。
        """
        try:
            if self.executor is not None:
                self.executor.shutdown()
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            finally:
                if self.context is not None and self.context.ok():
                    self.context.shutdown()

    def spin(self, seconds):
        """在调用线程驱动一次 ROS 回调，预算不抢占已开始的回调.

        Args:
            seconds: 传给 spin_once 的等待预算，单位为秒。

        Raises:
            Exception: executor 状态或回调执行失败。
        """
        self.executor.spin_once(timeout_sec=seconds)

    def publish(self):
        """依次向全部配置发布器提交默认零 Twist，不等待 DDS 确认.

        Raises:
            Exception: 消息构造或发布失败，后续发布器可能尚未尝试。
        """
        for publisher in self.publishers:
            publisher.publish(self.message_type())

    def receiver_count(self):
        """汇总各话题当前发现的订阅数量，不验证接收者身份或实际投递.

        Returns:
            各发布器发现数量之和，同一接收者可被不同话题重复计数。

        Raises:
            Exception: 发布器查询失败。
        """
        return sum(publisher.get_subscription_count() for publisher in self.publishers)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """在有界发现等待后尝试发布五次零速，并按既有规则报告退出状态.

    验证绝对话题名与 0 到 2 秒发现预算后才创建 ROS 传输；话题去重保留顺序。
    没有订阅者仅告警，不证明投递或实体停止。单次 spin、初始化与资源关闭可能
    超过请求预算，本函数不承诺总硬期限，也不在此检查导航租约或 ROS domain。

    Args:
        argv: 显式参数序列；None 时读取当前进程参数。

    Returns:
        尝试完成为 0，普通操作错误为 2，捕获的 Ctrl+C 为 130。

    Raises:
        SystemExit: 帮助或参数校验结束，在 ROS 初始化之前发生。
    """
    parser = argparse.ArgumentParser(description=_CLI_DESCRIPTION)
    parser.add_argument('--topic', action='append', required=True)
    parser.add_argument('--discovery-sec', type=float, default=1.0)
    args = parser.parse_args(argv)
    if any(not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', topic)
           for topic in args.topic):
        parser.error('topics must be absolute ROS names')
    if not math.isfinite(args.discovery_sec) or not 0.0 <= args.discovery_sec <= 2.0:
        parser.error('discovery-sec must be finite and between zero and two')
    try:
        with _RosZeros(tuple(dict.fromkeys(args.topic))) as transport:
            deadline = time.monotonic() + args.discovery_sec
            while time.monotonic() < deadline:
                transport.spin(min(0.05, max(0.0, deadline - time.monotonic())))
            receivers = transport.receiver_count()
            for _ in range(5):
                transport.publish()
                transport.spin(0.05)
        if not receivers:
            print('WARN zero command had no discovered receivers')
        print('zero messages published; physical stop is not verified', flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f'zero command failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
