#!/usr/bin/env python3
"""两段式对齐入位编排节点（2026-08-31）。

把一个目标拆成"预备点 -> 入位"两段，入位后用 TF 实测误差决定是否补旋转。

    收到 /goal_pose (map 系) ->
      预备点 = 目标位置 - approach_dist * [cos(goal_yaw), sin(goal_yaw)]
               朝向 = goal_yaw
      静态地图校验预备点的完整车身轮廓；有占用格或超出地图就拒绝
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

2026-09-15: 另开 **dock_to_pose 动作接口**(NavigateToPose 类型, 参数 action_name),
  供 navigation_executor 把 agent 下发的精准入位 POI 交给本节点。编排与话题入口完全
  相同; 动作接口额外提供: 结果(SUCCEEDED=入位达标, ABORTED=失败)、取消(当前阶段的
  nav2 目标随之取消, 不再下发后续阶段)、feedback(转发阶段目标的 feedback, 阶段1 的
  剩余距离加上 approach_dist)。执行期间再来的目标(话题或动作)一律拒绝。
"""
import math
import threading
import time
import traceback

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from david_nav.dock_approach import FootprintCheck, OccupancyGridView
from david_nav.dock_approach import check_footprint, parse_footprint
from david_nav.dock_finish import DockTolerance, finish_docking
from david_nav.dock_settle import SettleSpec


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# dock_finish 的失败判词; 其余 outcome 都是入位达标(见 execute_docking 末尾的 report)。
DOCK_FAILURE_REASONS = ('no_pose', 'not_settled', 'xy_out_of_tol', 'yaw_not_converged')
DEFAULT_FOOTPRINT = '[[0.255, 0.30], [0.255, -0.30], [-0.465, -0.30], [-0.465, 0.30]]'


def wait_done(fut, timeout_sec, should_cancel=None, poll_sec=0.1):
    """等 future 完成(自己不 spin)。True=完成; False=超时或 should_cancel() 为真。"""
    ev = threading.Event()
    fut.add_done_callback(lambda _f: ev.set())
    if fut.done():  # add_done_callback 之前就完成的竞态
        return True
    if should_cancel is None:
        return ev.wait(timeout_sec)
    deadline = time.monotonic() + timeout_sec
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        if ev.wait(min(poll_sec, remaining)):
            return True
        if should_cancel():
            return False


def stage_feedback(inner, extra_distance):
    """阶段目标(nav2)的 feedback -> 入位动作的 feedback: 剩余距离加上后续阶段的距离。"""
    feedback = NavigateToPose.Feedback()
    feedback.current_pose = inner.current_pose
    feedback.navigation_time = inner.navigation_time
    feedback.estimated_time_remaining = inner.estimated_time_remaining
    feedback.number_of_recoveries = inner.number_of_recoveries
    feedback.distance_remaining = float(inner.distance_remaining) + float(extra_distance)
    return feedback


class DockNavigator(Node):
    def __init__(self):
        super().__init__('dock_navigator')
        self.declare_parameter('approach_dist', 0.5)
        self.declare_parameter('xy_ok', 0.10)
        self.declare_parameter('yaw_ok', 0.087)
        self.declare_parameter('max_yaw_retries', 2)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('action_name', 'dock_to_pose')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('nav_timeout_sec', 120.0)
        self.declare_parameter('spin_timeout_sec', 30.0)
        # 预备点在发给 Nav2 前必须通过静态地图完整 footprint 校验。
        self.declare_parameter('static_map_topic', '/map')
        self.declare_parameter('static_occupied_threshold', 50)
        self.declare_parameter('footprint', DEFAULT_FOOTPRINT)
        # ---- 停稳判据（2026-09-10 新增）----
        # 实测静态定位抖动 yaw std 仅 0.068°，20Hz 下表观角速度约 0.02-0.05 rad/s，
        # 所以 max_rate 不能定得比这更严，否则永远等不到"停稳"。
        self.declare_parameter('settle_max_rate_rad_s', 0.05)
        self.declare_parameter('settle_window_sec', 0.4)
        self.declare_parameter('settle_timeout_sec', 8.0)

        g = self.get_parameter
        self.approach_dist = g('approach_dist').value
        self.xy_ok = g('xy_ok').value
        self.yaw_ok = g('yaw_ok').value
        self.max_yaw_retries = int(g('max_yaw_retries').value)
        self.base_frame = g('robot_base_frame').value
        self.global_frame = g('global_frame').value
        self.nav_timeout = g('nav_timeout_sec').value
        self.spin_timeout = g('spin_timeout_sec').value
        self.static_occupied_threshold = int(g('static_occupied_threshold').value)
        self.footprint = parse_footprint(g('footprint').value)
        self.settle_timeout = g('settle_timeout_sec').value
        self.settle_spec = SettleSpec(
            max_rate_rad_s=g('settle_max_rate_rad_s').value,
            window_sec=g('settle_window_sec').value)
        self.dock_tol = DockTolerance(xy_ok=self.xy_ok, yaw_ok=self.yaw_ok,
                                      max_yaw_retries=self.max_yaw_retries)
        self._spin_idx = 0

        self.cbg = ReentrantCallbackGroup()
        self._static_map = None
        self._static_map_lock = threading.Lock()
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, g('static_map_topic').value, self._on_static_map,
            map_qos, callback_group=self.cbg)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                    callback_group=self.cbg)
        self.spin_cli = ActionClient(self, Spin, 'spin', callback_group=self.cbg)
        self.create_subscription(PoseStamped, g('goal_topic').value,
                                 self.on_goal, 1, callback_group=self.cbg)
        self._busy = threading.Lock()
        # 动作接口执行期间的取消判据; 话题入口为 None(不可取消, 行为与原来一致)。
        self._cancel_check = None
        self._last_run_canceled = False
        self.dock_server = ActionServer(
            self, NavigateToPose, g('action_name').value,
            execute_callback=self._execute_dock_action,
            goal_callback=self._on_dock_goal_request,
            cancel_callback=self._on_dock_cancel_request,
            callback_group=self.cbg)

        self.get_logger().info(
            '[dock] 就绪: approach_dist=%.2fm xy_ok=%.3fm yaw_ok=%.3frad(%.1f°) '
            'retries=%d 目标话题=%s'
            % (self.approach_dist, self.xy_ok, self.yaw_ok,
               math.degrees(self.yaw_ok), self.max_yaw_retries,
               g('goal_topic').value))
        self.get_logger().info(
            '[dock] 动作接口: %s（NavigateToPose 类型，供 navigation_executor 调用）'
            % g('action_name').value)
        self.get_logger().info(
            '[dock] 预备点安全校验: 静态地图=%s occupied>=%d footprint=%s'
            % (g('static_map_topic').value, self.static_occupied_threshold,
               self.footprint))

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

    def _on_static_map(self, message):
        """缓存 transient-local 静态地图，供 dock 工作线程做预备点校验。"""
        view = OccupancyGridView.from_message(message)
        with self._static_map_lock:
            self._static_map = view

    def _check_approach_footprint(self, x, y, yaw):
        """检查预备点的完整车身轮廓，地图未就绪或帧不一致时保守拒绝。"""
        with self._static_map_lock:
            grid = self._static_map
        if grid is None:
            return FootprintCheck(False, 'static_map_not_received')
        if grid.frame_id != self.global_frame:
            return FootprintCheck(
                False, 'static_map_frame_mismatch:%s!=%s'
                % (grid.frame_id, self.global_frame))
        return check_footprint(
            grid, self.footprint, (x, y, yaw), self.static_occupied_threshold)

    def wait_future(self, fut, timeout_sec, should_cancel=None):
        """等 future 完成，但**自己绝不 spin** —— 由 main() 里的执行器负责转。

        ⚠ 这里以前用的是 rclpy.spin_until_future_complete(self, fut)。那个函数会把
        本节点 add_node 到 rclpy 的**全局 SingleThreadedExecutor**，而 main() 里的
        MultiThreadedExecutor 同时还持有这个节点、并且已经把 action client 的实体
        注册进了它自己的 wait set。两个执行器对同一个 action client 各建一套
        wait set，取就绪实体时下标对不上，直接抛：

            RCLError: Failed to get number of ready entities for action client:
            wait set index for status subscription is out of bounds,
            at ./src/rcl_action/action_client.c:623

        进程 exit 1 —— 2026-09-08 实机第一个 dock 目标就是这么死的：bt_navigator
        刚打印 "Begin navigating" 100ms 后就变成 "Client requested to cancel"，
        rviz 里能看到路径、车一步不动。

        换成事件等待：future 完成回调由本节点自己的执行器触发，这里只阻塞在
        threading.Event 上，不再碰任何 wait set。返回 True=完成，False=超时。
        """
        return wait_done(fut, timeout_sec, should_cancel)

    def run_action(self, client, goal_msg, label, timeout, feedback_callback=None):
        self._last_run_canceled = False
        should_cancel = self._cancel_check
        if should_cancel is not None and should_cancel():
            self.get_logger().warn('[dock] %s: 已收到取消请求，不再下发' % label)
            self._last_run_canceled = True
            return False
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('[dock] %s: action server 不可用' % label)
            return False
        send = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        if not self.wait_future(send, 10.0):
            self.get_logger().error('[dock] %s: 发目标后 10s 没有应答' % label)
            return False
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('[dock] %s: 目标被拒绝' % label)
            return False
        res_fut = handle.get_result_async()
        if not self.wait_future(res_fut, timeout, should_cancel):
            if should_cancel is not None and should_cancel():
                self.get_logger().warn('[dock] %s: 收到取消请求，取消当前目标' % label)
                self._last_run_canceled = True
                cancel = handle.cancel_goal_async()
                self.wait_future(cancel, 5.0)
                return False
            self.get_logger().error('[dock] %s: 超时 %.0fs，取消' % (label, timeout))
            cancel = handle.cancel_goal_async()
            self.wait_future(cancel, 5.0)
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
        # 编排是阻塞的（单段最长等 nav_timeout_sec 秒），不能占着执行器的回调线程：
        # 本节点的 action 应答、TF 回调都靠同一个执行器转，占着就是自己等自己。
        # 丢给独立工作线程跑，回调立即返回。
        threading.Thread(target=self._docking_worker, args=(msg,),
                         name='dock_worker', daemon=True).start()

    def _docking_worker(self, msg):
        try:
            self.execute_docking(msg)
        except Exception:  # noqa: BLE001
            self.get_logger().error('[dock] 编排线程异常，本次目标放弃：\n%s'
                                    % traceback.format_exc())
        finally:
            self._busy.release()

    # ---------- 动作接口 dock_to_pose（2026-09-15）----------
    def _on_dock_goal_request(self, goal_request):
        if self._busy.locked():
            self.get_logger().warn('[dock] 上一个目标还在执行，拒绝动作目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_dock_cancel_request(self, goal_handle):
        self.get_logger().warn('[dock] 收到动作取消请求')
        return CancelResponse.ACCEPT

    def _execute_dock_action(self, goal_handle):
        result = NavigateToPose.Result()
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn('[dock] 上一个目标还在执行，动作目标放弃')
            goal_handle.abort()
            return result
        outcome = 'failed'
        self._cancel_check = lambda: goal_handle.is_cancel_requested
        try:
            outcome = self.execute_docking(goal_handle.request.pose,
                                           publish_feedback=goal_handle.publish_feedback)
        except Exception:  # noqa: BLE001
            self.get_logger().error('[dock] 动作目标执行异常，本次目标放弃：\n%s'
                                    % traceback.format_exc())
        finally:
            self._cancel_check = None
            self._busy.release()
        if outcome == 'succeeded':
            goal_handle.succeed()
        elif outcome == 'canceled' and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    # ⚠ 方法名不能叫 handle: rclpy.node.Node.handle 是 property(底层 rcl_node_t),
    #   子类用普通方法覆盖它会让 Node.__init__ 里的 `with self.handle:` 拿到绑定方法,
    #   报 AttributeError: __enter__ —— 节点根本建不出来。
    def execute_docking(self, msg, publish_feedback=None):
        """两段式入位。返回 'succeeded' / 'failed' / 'canceled'(取消只可能来自动作接口)。"""
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

        approach_check = self._check_approach_footprint(px, py, gyaw)
        if not approach_check.clear:
            detail = approach_check.reason
            if approach_check.occupied_cell is not None:
                mx, my, value = approach_check.occupied_cell
                detail += ' cell=(%d,%d) value=%d' % (mx, my, value)
            self.report(False, '预备点车身轮廓静态地图校验失败: %s' % detail)
            return 'failed'
        self.get_logger().info('[dock] 预备点车身轮廓静态地图校验通过')

        g1 = NavigateToPose.Goal()
        g1.pose = self.make_pose(px, py, gyaw)
        if not self.run_action(self.nav_cli, g1, '阶段1(预备点)', self.nav_timeout,
                               self._feedback_forwarder(publish_feedback, self.approach_dist)):
            if self._last_run_canceled:
                self.get_logger().warn('[dock] ===== 已取消: 阶段1 被取消，不做入位')
                return 'canceled'
            self.report(False, '阶段1 未到预备点，放弃（不做入位）')
            return 'failed'

        self.get_logger().info('[dock] 阶段2 入位 -> (%.3f, %.3f) yaw=%.1f°'
                               % (gx, gy, math.degrees(gyaw)))
        g2 = NavigateToPose.Goal()
        g2.pose = self.make_pose(gx, gy, gyaw)
        stage2_ok = self.run_action(self.nav_cli, g2, '阶段2(入位)', self.nav_timeout,
                                    self._feedback_forwarder(publish_feedback, 0.0))
        if self._last_run_canceled:
            self.get_logger().warn('[dock] ===== 已取消: 阶段2 被取消，不做判定')
            return 'canceled'

        if not stage2_ok:
            self.get_logger().warn('[dock] 阶段2 未 SUCCEEDED，仍按停稳后的实测位姿判定')

        # ★ 2026-09-10：阶段2 之后的测量/判定/补偿整段交给 dock_finish，它会**先等车
        #   真正停稳**。原来在这里直接 pose_now()，而阶段2 报 SUCCEEDED 时车仍以
        #   0.55rad/s 在转（末端 RotateToGoal 满速收朝向的尾巴），测到的是运动中的
        #   瞬时 yaw；据此下发的 Spin 又被 nav2 的 abs(relative_yaw) 判据 100ms 秒过，
        #   两次补偿把误差从 6.8° 一路推到 19.6°。dock_bench 5 轮冒烟 3 轮栽在这。
        outcome = finish_docking(
            goal, self.pose_now, self._spin_by, self.dock_tol, self.settle_spec,
            settle_timeout_sec=self.settle_timeout, on_event=self._log_dock_event)

        if outcome.reason == 'no_pose':
            self.report(False, '入位后取不到位姿，无法判定')
        elif outcome.reason == 'not_settled':
            self.report(False, '车未停稳（%.1fs 内角速度未降到 %.3frad/s 以下），'
                               '按约定不做旋转补偿'
                        % (self.settle_timeout, self.settle_spec.max_rate_rad_s),
                        outcome.pose, goal)
        elif outcome.reason == 'xy_out_of_tol':
            self.report(False, 'xy 超差 %.3f > %.3f，按约定不再下发位移目标'
                        % (outcome.exy, self.xy_ok), outcome.pose, goal)
        elif outcome.reason == 'yaw_not_converged':
            self.report(False, 'yaw 经 %d 次旋转仍未收敛' % outcome.n_spin,
                        outcome.pose, goal)
        elif outcome.n_spin == 0:
            self.report(True, '一次入位达标', outcome.pose, goal)
        else:
            self.report(True, '第%d次旋转后达标' % outcome.n_spin, outcome.pose, goal)
        if outcome.reason in DOCK_FAILURE_REASONS:
            return 'canceled' if self._last_run_canceled else 'failed'
        return 'succeeded'

    def _feedback_forwarder(self, publish_feedback, extra_distance):
        """动作接口: 把阶段目标的 feedback 转发给入位动作的调用方; 话题入口不转发。"""
        if publish_feedback is None:
            return None

        def forward(feedback_msg):
            try:
                publish_feedback(stage_feedback(feedback_msg.feedback, extra_distance))
            except Exception:  # noqa: BLE001  转发失败不能影响入位本身
                pass
        return forward

    # ---------- 供 dock_finish 注入的两个副作用 ----------
    def _spin_by(self, target_yaw):
        sg = Spin.Goal()
        sg.target_yaw = float(target_yaw)
        sg.time_allowance = rclpy.duration.Duration(
            seconds=self.spin_timeout).to_msg()
        return self.run_action(self.spin_cli, sg, 'Spin#%d' % self._spin_idx,
                               self.spin_timeout + 5.0)

    def _log_dock_event(self, name, **kw):
        """把 dock_finish 的过程打进 /rosout。

        ⚠ 这些字符串是 scripts/dock_bench/dock_common.py 解析阶段边界和失败分类的
        **接口**，改动必须同步那边的正则与判词表。
        """
        if name == 'measured':
            self.get_logger().info(
                '[dock] 入位实测 exy=%.3fm eyaw=%.1f° (停稳=%s)'
                % (kw['exy'], math.degrees(kw['eyaw']),
                   '是' if kw['settled'] else '否'))
        elif name == 'spin':
            self._spin_idx = kw['index']
            self.get_logger().info('[dock] 第%d次纯旋转修正: Spin(%.1f°)'
                                   % (kw['index'], math.degrees(kw['target_yaw'])))
        elif name == 'after_spin':
            self.get_logger().info(
                '[dock] 旋转后 exy=%.3fm eyaw=%.1f° (停稳=%s)'
                % (kw['exy'], math.degrees(kw['eyaw']),
                   '是' if kw['settled'] else '否'))
        elif name == 'no_pose':
            self.get_logger().warn('[dock] 停稳等待期间取不到位姿')


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
