"""Fake Nav2 action servers and robot TF for sandbox checks (DDS domain 99 only).

Serves ``navigate_to_pose`` (records every goal, publishes feedback with
distance_remaining=1.0, succeeds after ``--delay`` seconds unless canceled)
and ``spin`` (succeeds at once), and broadcasts map->base_footprint at 20 Hz
at the given pose. Never publishes velocity commands.
"""

import argparse
import math
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class FakeNav2Servers(Node):
    """Fake navigate_to_pose and spin action servers plus a fixed robot pose."""

    def __init__(self, options):
        """Initialize the fake servers from parsed command-line options.

        Args:
            options: Namespace containing the log path, response delay, and
                fixed robot pose.
        """
        super().__init__('fake_nav2_servers')
        self.fake_options = options
        self.fake_lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self.nav_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute_navigation,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group)
        self.spin_server = ActionServer(
            self, Spin, 'spin', execute_callback=self.execute_spin, callback_group=group)
        self.pose_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_pose)
        if options.map_mode != 'none':
            qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE)
            self.static_map_publisher = self.create_publisher(OccupancyGrid, '/map', qos)
            self.static_map = OccupancyGrid()
            self.static_map.header.frame_id = 'map'
            self.static_map.info.resolution = 0.1
            self.static_map.info.width = 200
            self.static_map.info.height = 200
            self.static_map.info.origin.position.x = -10.0
            self.static_map.info.origin.position.y = -10.0
            self.static_map.info.origin.orientation.w = 1.0
            cells = [0] * 40000
            if options.map_mode == 'occupied_approach':
                # Occupy a body corner, not the approach point's center cell.
                c, s = math.cos(options.yaw), math.sin(options.yaw)
                x = options.x - 0.5 * c + 0.20 * c - 0.20 * s
                y = options.y - 0.5 * s + 0.20 * s + 0.20 * c
                col, row = int((x + 10.0) / 0.1), int((y + 10.0) / 0.1)
                cells[row * 200 + col] = 100
            self.static_map.data = cells
            self.publish_static_map()
            self.create_timer(0.2, self.publish_static_map)

    def publish_static_map(self):
        """Publish a latched free map or an occupied approach-footprint corner."""
        self.static_map.header.stamp = self.get_clock().now().to_msg()
        self.static_map_publisher.publish(self.static_map)

    def record(self, line):
        """Append one action-server event to the sandbox log.

        Args:
            line: Event text to append.
        """
        with self.fake_lock, open(self.fake_options.log, 'a', encoding='utf-8') as handle:
            handle.write(line + '\n')

    def publish_pose(self):
        """Broadcast the configured fixed robot pose."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.fake_options.x
        transform.transform.translation.y = self.fake_options.y
        transform.transform.rotation.z = math.sin(self.fake_options.yaw / 2.0)
        transform.transform.rotation.w = math.cos(self.fake_options.yaw / 2.0)
        self.pose_broadcaster.sendTransform(transform)

    def execute_navigation(self, goal_handle):
        """Execute a fake navigation goal and record its outcome.

        Args:
            goal_handle: ROS action goal handle for the navigation request.

        Returns:
            An empty navigation result after success or cancellation.
        """
        position = goal_handle.request.pose.pose.position
        self.record('goal %.3f %.3f' % (position.x, position.y))
        deadline = time.monotonic() + self.fake_options.delay
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.record('canceled')
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = 1.0
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        goal_handle.succeed()
        self.record('succeeded')
        return NavigateToPose.Result()

    def execute_spin(self, goal_handle):
        """Succeed a fake spin goal immediately.

        Args:
            goal_handle: ROS action goal handle for the spin request.

        Returns:
            An empty successful spin result.
        """
        goal_handle.succeed()
        return Spin.Result()


def main():
    """Run the fake Nav2 servers until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True)
    parser.add_argument('--delay', type=float, default=0.5)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument(
        '--map-mode', choices=['free', 'occupied_approach', 'none'], default='free')
    options = parser.parse_args()
    rclpy.init()
    node = FakeNav2Servers(options)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
