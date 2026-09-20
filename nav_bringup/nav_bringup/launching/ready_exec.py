"""等导航动作服务端出现，然后不经 shell 直接替换本进程.

等待是只读的：只看服务端在不在，不下发任何目标，也不改变导航状态。
替换使用 exec，不新建子进程，因此成功路径不会返回。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Optional, Sequence


def wait_ready(action_name: str, timeout_sec: float) -> bool:
    """只读地等待动作服务端，绝不下发导航目标.

    Args:
        action_name: 用作就绪信号的 NavigateToPose 动作。
        timeout_sec: 最长墙钟等待秒数。

    Returns:
        仅当在超时或 ROS 关停之前发现服务端时为 True。发现服务端只说明它已注册，
        不证明导航能规划出路径或能到达任何目标。
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
    """过一道就绪门禁，然后 exec 执行已备好的那条命令.

    命令逐项传递，不经 shell 解释。

    Args:
        argv: 不含可执行文件名的参数；None 表示使用本进程参数。

    Returns:
        超时、被中断或 exec 失败时返回失败状态码；成功时本进程已被替换，
        因此根本不会返回。
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
