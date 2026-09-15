"""Bounded zero-only shutdown helper invoked by the owning startup supervisor.

There is deliberately no velocity argument. Publishing a zero Twist is an
attempt to stop, not evidence that a physical robot stopped. The supervisor
must own navigation before invoking this helper; rejected startup must not
publish into somebody else's graph. Imports do not initialize ROS.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from typing import Optional, Sequence


class _RosZeros:
    def __init__(self, topics):
        self.topics = topics
        self.context = None
        self.node = None
        self.executor = None
        self.publishers = []

    def __enter__(self):
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
        self.executor.spin_once(timeout_sec=seconds)

    def publish(self):
        for publisher in self.publishers:
            publisher.publish(self.message_type())

    def receiver_count(self):
        return sum(publisher.get_subscription_count() for publisher in self.publishers)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Publish five zero-only messages after bounded DDS discovery, then exit.

    Args:
        argv: Explicit arguments, or the current process arguments when None.

    Returns:
        Zero after a completed publication attempt, two on an operational error,
        or 130 on Ctrl+C. No subscribers produces a warning, not a stop guarantee.

    Raises:
        SystemExit: Invalid CLI arguments, before any ROS initialization.
    """
    parser = argparse.ArgumentParser(description=__doc__)
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
