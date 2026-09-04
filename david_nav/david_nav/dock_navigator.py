#!/usr/bin/env python3
"""两段式对齐入位编排节点（2026-08-31）。

把一个目标拆成"预备点 -> 入位"两段，入位后用 TF 实测误差决定是否补旋转。

    收到 /goal_pose (map 系) ->
      预备点 = 目标位置 - approach_dist * [cos(goal_yaw), sin(goal_yaw)]
               朝向 = goal_yaw
      阶段1  NavigateToPose(预备点)   允许位移
      阶段2  NavigateToPose(目标)     允许位移, 沿目标航向直线入位
      TF 实测 map->base_footprint:
        exy<=xy_ok 且 |eyaw|<=yaw_ok            -> 成功
        exy<=xy_ok 且 |eyaw|>yaw_ok             -> Spin(-eyaw) 纯旋转, 最多 max_yaw_retries 次
        exy>xy_ok                               -> 失败, 不再下发任何位移目标

为什么用 Spin 而不是再发一次导航目标: Spin 行为只发角速度, 不会平移, 这是
"入位失败后只能调角度不能位移"的硬保证。再发 NavigateToPose 则无法阻止它平移。

⚠ 话题分工, 别接错: bt_navigator 自己也订阅 /goal_pose, 若本节点也收 /goal_pose,
  一个目标会同时触发两条链路。本节点默认收 **/dock_goal**(由 launch 传参设定):
      /dock_goal -> dock_navigator  两段式入位(rviz "2D Goal Pose" 工具发这个)
      /goal_pose -> bt_navigator    单段直达, 留作 CLI 对照测试
  之所以用分话题而不是 remap: IncludeLaunchDescription 无法给被包含 launch 里的
  节点追加 remapping, 而改 remap 就得动生产 launch 文件。
"""
import math
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DockNavigator(Node):
    def __init__(self):
        super().__init__('dock_navigator')
        self.declare_parameter('approach_dist', 0.5)
        self.declare_parameter('xy_ok', 0.10)
        self.declare_parameter('yaw_ok', 0.087)
        self.declare_parameter('max_yaw_retries', 2)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('nav_timeout_sec', 120.0)
        self.declare_parameter('spin_timeout_sec', 30.0)

        g = self.get_parameter
        self.approach_dist = g('approach_dist').value
        self.xy_ok = g('xy_ok').value
        self.yaw_ok = g('yaw_ok').value
        self.max_yaw_retries = int(g('max_yaw_retries').value)
        self.base_frame = g('robot_base_frame').value
        self.global_frame = g('global_frame').value
        self.nav_timeout = g('nav_timeout_sec').value
        self.spin_timeout = g('spin_timeout_sec').value

        self.cbg = ReentrantCallbackGroup()
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                    callback_group=self.cbg)
        self.spin_cli = ActionClient(self, Spin, 'spin', callback_group=self.cbg)
        self.create_subscription(PoseStamped, g('goal_topic').value,
                                 self.on_goal, 1, callback_group=self.cbg)
        self._busy = threading.Lock()

        self.get_logger().info(
            '[dock] 就绪: approach_dist=%.2fm xy_ok=%.3fm yaw_ok=%.3frad(%.1f°) '
            'retries=%d 目标话题=%s'
            % (self.approach_dist, self.xy_ok, self.yaw_ok,
               math.degrees(self.yaw_ok), self.max_yaw_retries,
               g('goal_topic').value))

    # ---------- 工具 ----------
    def pose_now(self, timeout_sec=2.0):
        try:
            tr = self.tf_buf.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('[dock] 取位姿失败: %s' % e)
            return None
        t = tr.transform.translation
        return (t.x, t.y, yaw_of(tr.transform.rotation))

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.global_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def run_action(self, client, goal_msg, label, timeout):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[dock] %s: action server 不可用' % label)
            return False
        send = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('[dock] %s: 目标被拒绝' % label)
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        if not res_fut.done():
            self.get_logger().error('[dock] %s: 超时 %.0fs，取消' % (label, timeout))
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return False
        status = res_fut.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info('[dock] %s: %s' % (label, 'SUCCEEDED' if ok else
                                                  'status=%d' % status))
        return ok

    def report(self, ok, msg, p=None, goal=None):
        if p is not None and goal is not None:
            exy = math.hypot(p[0] - goal[0], p[1] - goal[1])
            eyaw = wrap(p[2] - goal[2])
            msg = '%s  (实测 exy=%.3fm eyaw=%.1f°)' % (msg, exy, math.degrees(eyaw))
        if ok:
            self.get_logger().info('[dock] ===== 成功: %s' % msg)
        else:
            self.get_logger().error('[dock] ===== 失败: %s' % msg)

    # ---------- 主流程 ----------
    def on_goal(self, msg):
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('[dock] 上一个目标还在执行，忽略本次')
            return
        try:
            self.execute_docking(msg)
        finally:
            self._busy.release()

    # ⚠ 方法名不能叫 handle: rclpy.node.Node.handle 是 property(底层 rcl_node_t),
    #   子类用普通方法覆盖它会让 Node.__init__ 里的 `with self.handle:` 拿到绑定方法,
    #   报 AttributeError: __enter__ —— 节点根本建不出来。
    def execute_docking(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gyaw = yaw_of(msg.pose.orientation)
        goal = (gx, gy, gyaw)
        self.get_logger().info(
            '[dock] ===== 收到目标 (%.3f, %.3f) yaw=%.1f°' % (gx, gy, math.degrees(gyaw)))

        px = gx - self.approach_dist * math.cos(gyaw)
        py = gy - self.approach_dist * math.sin(gyaw)
        self.get_logger().info(
            '[dock] 阶段1 预备点 (%.3f, %.3f) yaw=%.1f°（沿目标航向后退 %.2fm）'
            % (px, py, math.degrees(gyaw), self.approach_dist))

        g1 = NavigateToPose.Goal()
        g1.pose = self.make_pose(px, py, gyaw)
        if not self.run_action(self.nav_cli, g1, '阶段1(预备点)', self.nav_timeout):
            self.report(False, '阶段1 未到预备点，放弃（不做入位）')
            return

        self.get_logger().info('[dock] 阶段2 入位 -> (%.3f, %.3f) yaw=%.1f°'
                               % (gx, gy, math.degrees(gyaw)))
        g2 = NavigateToPose.Goal()
        g2.pose = self.make_pose(gx, gy, gyaw)
        stage2_ok = self.run_action(self.nav_cli, g2, '阶段2(入位)', self.nav_timeout)

        p = self.pose_now()
        if p is None:
            self.report(False, '入位后取不到位姿，无法判定')
            return
        exy = math.hypot(p[0] - gx, p[1] - gy)
        eyaw = wrap(p[2] - gyaw)
        self.get_logger().info('[dock] 入位实测 exy=%.3fm eyaw=%.1f° (阶段2 %s)'
                               % (exy, math.degrees(eyaw),
                                  'SUCCEEDED' if stage2_ok else 'FAILED'))

        if exy <= self.xy_ok and abs(eyaw) <= self.yaw_ok:
            self.report(True, '一次入位达标', p, goal)
            return

        if exy > self.xy_ok:
            # 位置没到就不再挪 —— 用户明确要求: 入位失败后只能调角度, 不能位移
            self.report(False, 'xy 超差 %.3f > %.3f，按约定不再下发位移目标'
                        % (exy, self.xy_ok), p, goal)
            return

        # xy 达标、只差朝向 -> 纯旋转补偿
        for i in range(1, self.max_yaw_retries + 1):
            self.get_logger().info('[dock] 第%d次纯旋转修正: Spin(%.1f°)'
                                   % (i, math.degrees(-eyaw)))
            sg = Spin.Goal()
            sg.target_yaw = float(-eyaw)
            sg.time_allowance = rclpy.duration.Duration(
                seconds=self.spin_timeout).to_msg()
            self.run_action(self.spin_cli, sg, 'Spin#%d' % i, self.spin_timeout + 5.0)
            p = self.pose_now()
            if p is None:
                self.report(False, '旋转后取不到位姿')
                return
            exy = math.hypot(p[0] - gx, p[1] - gy)
            eyaw = wrap(p[2] - gyaw)
            self.get_logger().info('[dock] 旋转后 exy=%.3fm eyaw=%.1f°'
                                   % (exy, math.degrees(eyaw)))
            if abs(eyaw) <= self.yaw_ok:
                self.report(True, '第%d次旋转后达标' % i, p, goal)
                return
        self.report(False, 'yaw 经 %d 次旋转仍未收敛' % self.max_yaw_retries, p, goal)


def main():
    rclpy.init()
    node = DockNavigator()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
