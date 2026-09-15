"""Keep ordinary command-package tests disconnected from the ROS graph."""

import pytest
import rclpy
from rclpy.node import Node


@pytest.fixture(autouse=True)
def no_ros_runtime(monkeypatch):
    """Reject ROS initialization and nodes; message objects and pure methods are safe."""
    def forbidden(*args, **kwargs):
        pytest.fail("ordinary tests must not initialize ROS or create nodes")

    monkeypatch.setattr(rclpy, "init", forbidden)
    monkeypatch.setattr(rclpy, "create_node", forbidden)
    monkeypatch.setattr(Node, "__init__", forbidden)
