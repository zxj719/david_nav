"""Publish synthetic interface data only in domain 99, never command velocities."""

import argparse
import os
from pathlib import Path
import signal

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_ros import StaticTransformBroadcaster

from nav_perception_iface.configuration import load_contract_profile


def main() -> None:
    """Expose test-only data and TF; a command subscriber never sends a command.

    Raises:
        ValueError: The process is outside the designated domain-99 sandbox.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--variant', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '99':
        raise ValueError('fixture requires domain 99')
    profile = load_contract_profile(args.profile)
    stopped = [False]

    def stop(*_):
        stopped[0] = True

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        if rclpy.get_default_context().get_domain_id() != 99:
            raise ValueError('actual context must also be domain 99')
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        node = rclpy.create_node('synthetic_contract_provider')
        frames = profile['frames']
        odometry = [node.create_publisher(Odometry, topic, qos_profile_sensor_data) for topic in (
            profile['localization']['odom_topic'], profile['chassis']['odom_topic'])]
        if args.variant != 'no_driver_sub':
            message_type = Odometry if args.variant == 'wrong_driver_type' else Twist
            node.create_subscription(message_type, profile['chassis']['cmd_topic'], lambda _: None,
                                     10)
        broadcaster = StaticTransformBroadcaster(node)
        pairs = [(frames['body'], frames['nav_base'])]
        if args.variant not in ('waiting', 'interrupt'):
            pairs.append((frames['map'], frames['odom']))
        if args.variant != 'missing_body':
            pairs.append((frames['odom'], frames['body']))
        transforms = []
        for parent, child in pairs:
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        broadcaster.sendTransform(transforms)

        def publish():
            message = Odometry()
            message.header.frame_id = frames['odom']
            message.child_frame_id = frames['body']
            message.header.stamp = node.get_clock().now().to_msg()
            message.pose.pose.orientation.w = 1.0
            for index, publisher in enumerate(odometry):
                if args.variant != 'silent' or index != 0:
                    publisher.publish(message)
        node.create_timer(0.05, publish)
        print('contract fixture ready', flush=True)
        while not stopped[0]:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
