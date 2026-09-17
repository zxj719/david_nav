"""管理两个导航仿真节点共用的进程内 ROS 初始化、运行和收尾."""

import os
from typing import Callable, Optional, Sequence

import rclpy
from rclpy.node import Node

from nav_sim.model import require_sandbox


def require_node_sandbox(node: Node) -> None:
    """核对节点实际上下文为域 99，不只依赖进程环境变量.

    Args:
        node: 刚创建且尚未创建业务端点或定时器的节点。

    Raises:
        RuntimeError: 实际域不是 99；先尝试销毁节点，销毁失败则传播其异常。
    """
    if node.context.get_domain_id() != 99:
        node.destroy_node()
        raise RuntimeError('nav_sim node context must use ROS domain 99')


def run(factory: Callable[[], Node], args: Optional[Sequence[str]] = None) -> None:
    """在域 99 环境运行假节点，退出时顺序尝试销毁节点和关闭默认上下文.

    初始化发生在 try 之前；节点销毁抛异常时不会继续调用 shutdown。
    这里使用进程默认上下文，不创建独立 Context，也不清理其它进程。

    Args:
        factory: 初始化后调用的节点构造器，节点自身还须核对实际域。
        args: 可选 ROS 命令行参数。

    Raises:
        RuntimeError: 未显式设置域 99；构造、运行或收尾的其它异常按原样传播。
    """
    require_sandbox(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
