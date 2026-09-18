# Copyright 2026 xingjian zhang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
