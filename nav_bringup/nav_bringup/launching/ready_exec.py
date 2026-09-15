"""Wait for the navigation action server, then replace this process without a shell."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Optional, Sequence


def wait_ready(action_name: str, timeout_sec: float) -> bool:
    """Wait read-only for an action server; never submit a navigation goal.

    Args:
        action_name: NavigateToPose action whose server signals readiness.
        timeout_sec: Maximum wall-clock wait in seconds.

    Returns:
        True only if the server is discovered before timeout or ROS shutdown.
    """
    from nav2_msgs.action import NavigateToPose
    import rclpy
    from rclpy.action import ActionClient

    rclpy.init(args=[])
    node = None
    client = None
    try:
        node = rclpy.create_node('nav_launch_ready_gate')
        client = ActionClient(node, NavigateToPose, action_name)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if client.wait_for_server(timeout_sec=min(0.25, remaining)):
                return rclpy.ok()
        return False
    finally:
        if client is not None:
            client.destroy()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the readiness gate and exec the exact prepared command.

    Args:
        argv: Arguments excluding executable name, or None for process arguments.

    Returns:
        Failure status on timeout, interruption or exec failure; success replaces
        the process and therefore does not return.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--action', default='/navigate_to_pose')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not args.action:
        parser.error('timeout must be positive/finite and action nonempty')
    if not command or not os.path.isabs(command[0]):
        parser.error('an absolute executable after -- is required')
    try:
        if not wait_ready(args.action, args.timeout):
            print('navigation did not become ready; executor was not started', file=sys.stderr)
            return 1
        os.execv(command[0], command)
    except KeyboardInterrupt:
        return 130
    except OSError as error:
        print(f'executor could not start: {error}', file=sys.stderr)
        return 127
    return 0
