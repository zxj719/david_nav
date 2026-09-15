"""Expose prepared parameters and static TF in domain 99 without command publication."""

import argparse
from pathlib import Path
import signal

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message
from tf2_ros import StaticTransformBroadcaster

from nav_bringup.navcfg.live_inputs import load_live_spec
from nav_perception_iface.configuration import load_contract_profile


def main() -> None:
    """Serve only sandbox-owned mock nodes, parameter values, endpoints and static TF."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--bad-parameter', action='store_true')
    parser.add_argument('--no-localization', action='store_true')
    parser.add_argument('--silent-odom', action='store_true')
    args = parser.parse_args()
    spec, _ = load_live_spec(args.run_dir)
    profile = load_contract_profile(args.run_dir / 'params/inputs/robot.yaml',
                                    args.run_dir / 'params/inputs/mode.yaml')
    stopped = [False]

    def stop(*_):
        stopped[0] = True

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        for qualified, parameters in spec.parameters.items():
            namespace, _, name = qualified.rpartition('/')
            values = dict(parameters)
            if args.bad_parameter and qualified == '/chassis_adapter':
                values['max_linear_x'] += 0.01
            node = rclpy.create_node(
                name, namespace=namespace or '/',
                parameter_overrides=[Parameter(key, value=value) for key, value in values.items()],
                automatically_declare_parameters_from_overrides=True)
            nodes.append(node)
            executor.add_node(node)
        observer = rclpy.create_node('live_fixture_graph')
        nodes.append(observer)
        executor.add_node(observer)
        # Only synthetic odometry is published; the command input remains passive.
        odometry = []
        for rule in spec.topics:
            publisher = observer.create_publisher(get_message(rule.message_type), rule.topic, 10)
            if rule.message_type == 'nav_msgs/msg/Odometry' and not (
                    args.silent_odom and rule.topic == profile['localization']['odom_topic']):
                odometry.append(publisher)
        observer.create_subscription(Twist, profile['chassis']['cmd_topic'], lambda _: None, 10)

        def publish_odometry():
            message = Odometry()
            message.header.stamp = observer.get_clock().now().to_msg()
            message.header.frame_id = spec.odom_frame
            for publisher in odometry:
                publisher.publish(message)

        observer.create_timer(0.05, publish_odometry)
        if not args.no_localization:
            broadcaster = StaticTransformBroadcaster(observer)
            transforms = []
            for parent, child in ((spec.map_frame, spec.odom_frame),
                                  (spec.odom_frame, profile['frames']['body']),
                                  (profile['frames']['body'], spec.nav_frame)):
                if parent == child:
                    continue
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.header.stamp = observer.get_clock().now().to_msg()
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            broadcaster.sendTransform(transforms)
        print('live fixture ready', flush=True)
        while rclpy.ok() and not stopped[0]:
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
