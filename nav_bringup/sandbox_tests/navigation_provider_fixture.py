"""Synthetic stationary localization for goal-free Nav2 startup checks.

Since 757cd1e the tour/precise costmaps use only static + inflation layers, so no
synthetic scan is published (the profile no longer has nav2.scan_topic).
"""

import argparse
import json
import os
from pathlib import Path
import signal

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster
import yaml


def main() -> None:
    """Publish synthetic sensor data in domain 99, never goals or velocity commands."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '99':
        raise ValueError('synthetic navigation provider requires domain 99')
    profile = yaml.safe_load(args.profile.read_text())
    frames = profile['frames']
    stopped = [False]
    counts = {'commands': 0, 'nonzero': 0}

    def stop(*_):
        stopped[0] = True

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = rclpy.create_node('stationary_navigation_provider')
    try:
        if node.context.get_domain_id() != 99:
            raise ValueError('unexpected actual ROS domain')
        odometry = node.create_publisher(Odometry, profile['localization']['odom_topic'], 10)
        broadcaster = TransformBroadcaster(node)
        transforms = []
        for parent, child in ((frames['map'], frames['odom']), (frames['odom'], frames['body'])):
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.0
            if parent == frames['map']:
                transform.transform.translation.x = 3.0
                transform.transform.translation.y = 3.0
            transforms.append(transform)

        def command(message):
            counts['commands'] += 1
            if any(value != 0.0 for vector in (message.linear, message.angular)
                   for value in (vector.x, vector.y, vector.z)):
                counts['nonzero'] += 1
                raise RuntimeError('unexpected nonzero command during goal-free startup')

        node.create_subscription(Twist, profile['chassis']['cmd_topic'], command, 10)

        def publish():
            stamp = node.get_clock().now().to_msg()
            for transform in transforms:
                transform.header.stamp = stamp
            broadcaster.sendTransform(transforms)
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = frames['odom']
            message.child_frame_id = frames['body']
            message.pose.pose.orientation.w = 1.0
            odometry.publish(message)

        node.create_timer(0.05, publish)
        print('stationary navigation provider ready', flush=True)
        while rclpy.ok() and not stopped[0]:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(counts), flush=True)


if __name__ == '__main__':
    main()
