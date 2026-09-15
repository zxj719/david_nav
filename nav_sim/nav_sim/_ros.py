"""Small ROS process lifecycle shared by the two simulation nodes."""

import os
from typing import Callable, Optional, Sequence

import rclpy
from rclpy.node import Node

from nav_sim.model import require_sandbox


def require_node_sandbox(node: Node) -> None:
    """Reject an explicitly supplied ROS context that bypasses the environment.

    Args:
        node: Node just created, before any endpoint or timer is constructed.

    Raises:
        RuntimeError: Actual context belongs to a domain other than 99.
    """
    if node.context.get_domain_id() != 99:
        node.destroy_node()
        raise RuntimeError('nav_sim node context must use ROS domain 99')


def run(factory: Callable[[], Node], args: Optional[Sequence[str]] = None) -> None:
    """Spin a node in domain 99 and always release only our own context.

    Args:
        factory: Node constructor invoked after initialization.
        args: Optional ROS command-line arguments.
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
