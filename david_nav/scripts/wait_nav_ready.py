#!/usr/bin/env python3
"""等 nav2 真正可用 —— 直接探 /navigate_to_pose action server，不经 ros2 CLI 守护进程。

为什么不用 `ros2 node list | grep bt_navigator`：
  该命令走 ros2 CLI 守护进程，而守护进程会用它【启动时】的环境建 DDS 参与者，
  且会僵死成 `xmlrpc.client.Fault: !rclpy.ok()`。此时它静默返空 ->
  start_navigation_headless.sh 判定 bt_navigator 没起来 -> 卡满 120s，
  而 nav2 其实好好在跑（rviz 里 costmap 正常刷新即铁证）。
  2026-09-07 / 09-08 连续两天复现，2026-08-25 也记过一次。

为什么探 action server 而不是节点名：
  节点名出现 ≠ 可以下发导航目标。bt_navigator 的 action server 只在 lifecycle
  activate 之后才注册，所以"server 可用"恰好等价于 executor 真正需要的条件，
  语义比节点名更准，还顺带覆盖了 lifecycle 没起来的情况。

退出码: 0=可用, 1=超时, 2=环境/参数错误
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="本次探测的最长等待秒数（脚本侧循环调用）")
    ap.add_argument("--action", default="/navigate_to_pose")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from nav2_msgs.action import NavigateToPose
    except Exception as e:  # noqa: BLE001
        print("wait_nav_ready: 无法导入 rclpy/nav2_msgs: %s" % e, file=sys.stderr)
        return 2

    rclpy.init()
    node = None
    try:
        node = Node("wait_nav_ready_probe")
        cli = ActionClient(node, NavigateToPose, args.action)
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            # wait_for_server 内部会 spin，这里切成小片以便及时返回
            if cli.wait_for_server(timeout_sec=0.5):
                if not args.quiet:
                    print("wait_nav_ready: %s action server 可用" % args.action)
                return 0
        if not args.quiet:
            print("wait_nav_ready: %.1fs 内 %s 仍不可用" % (args.timeout, args.action),
                  file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
