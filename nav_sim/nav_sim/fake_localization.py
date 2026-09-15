"""Publish only map-to-odom and odom-to-body TF from simulated odometry."""

import copy
import math
import os
from typing import Optional, Sequence

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from nav_sim import _ros
from nav_sim.model import localization_frames, require_sandbox


class FakeLocalization(Node):
    """Mirror integrated odometry into TF; URDF owns body-to-navigation TF."""

    def __init__(self) -> None:
        """Require robot frame configuration and subscribe to simulated odometry."""
        require_sandbox(os.environ)
        super().__init__('fake_localization')
        _ros.require_node_sandbox(self)
        self._map, self._odom, self._body = localization_frames(*(
            self.declare_parameter(name, '').value
            for name in ('map_frame', 'odom_frame', 'body_frame')))
        self._broadcaster = TransformBroadcaster(self)
        self._publisher = None
        output = self.declare_parameter('output_odom_topic', '').value
        source = self.declare_parameter('odom_topic', '/odom').value
        if output and self.resolve_topic_name(output) != self.resolve_topic_name(source):
            self._publisher = self.create_publisher(Odometry, output, 10)
        self._subscription = self.create_subscription(Odometry, source, self._odometry, 10)

    def _odometry(self, message: Odometry) -> None:
        if message.header.frame_id != self._odom or message.child_frame_id != self._body:
            self.get_logger().warning('simulated odometry frame mismatch; ignoring sample')
            return
        pose = message.pose.pose
        quaternion = pose.orientation
        components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if (not all(math.isfinite(value) for value in (
                pose.position.x, pose.position.y, pose.position.z, *components))
                or abs(sum(value * value for value in components) - 1.0) > 1e-6):
            self.get_logger().warning('invalid simulated odometry pose; ignoring sample')
            return
        fixed, moving = TransformStamped(), TransformStamped()
        fixed.header.stamp = message.header.stamp
        fixed.header.frame_id = self._map
        fixed.child_frame_id = self._odom
        fixed.transform.rotation.w = 1.0
        moving.header.stamp = message.header.stamp
        moving.header.frame_id = self._odom
        moving.child_frame_id = self._body
        moving.transform.translation.x = pose.position.x
        moving.transform.translation.y = pose.position.y
        moving.transform.translation.z = pose.position.z
        moving.transform.rotation = copy.deepcopy(pose.orientation)
        self._broadcaster.sendTransform([fixed, moving])
        if self._publisher is not None:
            self._publisher.publish(copy.deepcopy(message))


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run the isolated fake localization without launching a SLAM stack.

    Args:
        args: Optional ROS command-line arguments.
    """
    _ros.run(FakeLocalization, args)
