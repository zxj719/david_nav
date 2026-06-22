#!/usr/bin/env python3
"""waypoint_follower — Nav2 plans, position_controller drives.

Asks Nav2 to PLAN ONLY (ComputePathToPose), sparsifies the global path into
waypoints where every consecutive pair is a collision-free straight line
<= max_segment (default 3 m), then drives each segment with the existing
position_controller via its MoveTo action. The local controller (DWB) is
bypassed; blind straight-line driving is safe because each segment was
verified clear on the global costmap.

Static-map assumption: segments are validated once at plan time. No runtime
obstacle monitoring or replanning.

Interfaces:
  Action ~/navigate : nav2_msgs/action/NavigateToPose (reuses the type; this is
                      NOT nav2's bt_navigator — point tools here explicitly)
  Topic  ~/goal_pose: geometry_msgs/PoseStamped (convenience, map frame)
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from david_nav_msgs.action import MoveTo

from david_nav.grid_map_view import GridMapView
from david_nav.waypoint_sparsifier import (
    plan_waypoints,
    pose_window_stable,
    relative_target,
    relative_yaw,
)


def _yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class WaypointFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        self.declare_parameter("planner_action", "compute_path_to_pose")
        self.declare_parameter("move_action", "/position_controller/move")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("max_segment", 3.0)
        self.declare_parameter("los_cost_threshold", 99)
        self.declare_parameter("block_unknown", True)
        self.declare_parameter("intermediate_heading", False)
        self.declare_parameter("planner_server_timeout", 5.0)
        self.declare_parameter("move_server_timeout", 5.0)
        self.declare_parameter("costmap_wait", 5.0)
        # 分层容差(2026-06-10):中间 waypoint 是过路点,松;最终 goal 紧并校正
        # 朝向。发段前通过 position_controller 的 set_parameters 服务切换
        # (controller 每次接 goal 重读参数,所以设完下一段立即生效)。
        self.declare_parameter("wp_dist_tol", 0.8)     # m,中间 waypoint 位置容差
        self.declare_parameter("goal_dist_tol", 0.4)   # m,最终 goal 位置容差
        self.declare_parameter("goal_yaw_tol", 0.4)    # rad,最终 goal 朝向容差
        # 直行入位(2026-06-10):goal 正后方插进场点 A,最后一段沿最终朝向直入。
        # approach_dist=0 关闭。进场点容差要比普通 waypoint 紧,否则入位几何被
        # 0.8m 的松容差破坏(0.25m/0.8m ⇒ 入位方向误差 ≤ ~17°,终转 0.4rad 兜底)。
        self.declare_parameter("approach_dist", 0.8)       # m,进场点离 goal 距离
        self.declare_parameter("approach_dist_tol", 0.25)  # m,进场点位置容差
        # 定位收敛等待 + 末端闭环复查(2026-06-10):重定位在运动中/刚停时位姿
        # 是脏的,静止后才收敛(±1.5cm/0.1°)。段起点和末端复查都先等收敛。
        self.declare_parameter("pose_stable_window", 0.5)   # s,连续稳定窗口
        self.declare_parameter("pose_stable_eps_xy", 0.03)  # m,窗口内位置跨度
        self.declare_parameter("pose_stable_eps_yaw", 0.03) # rad,窗口内朝向跨度
        self.declare_parameter("pose_stable_timeout", 6.0)  # s,等不到就用最新值
        self.declare_parameter("final_check_retries", 2)    # 末端复查补走轮数,0=关
        # 终点补偿到 base_link(2026-06-22):控制/测量基于 robot_base_frame
        # (base_footprint=两轮轴中点=旋转中心),但它在 base_link 前方 ~0.165m。
        # 直接把 base_footprint 开到 goal 会让身体中心(base_link,即 SLAM/抓取基准)
        # 差这个偏置。开启后:把 goal 沿最终朝向前移 偏置量,使终转后 goal_ref_frame
        # (默认 base_link)恰落在 goal 上。偏置从 TF 实读 → 与 URDF 几何解耦。
        self.declare_parameter("compensate_goal_to_ref", True)
        self.declare_parameter("goal_ref_frame", "base_link")

        g = self.get_parameter
        self.planner_action = str(g("planner_action").value)
        self.move_action = str(g("move_action").value)
        self.costmap_topic = str(g("global_costmap_topic").value)
        self.map_frame = str(g("map_frame").value)
        self.robot_base_frame = str(g("robot_base_frame").value)
        self.planner_id = str(g("planner_id").value)
        self.max_segment = float(g("max_segment").value)
        self.los_threshold = int(g("los_cost_threshold").value)
        self.block_unknown = bool(g("block_unknown").value)
        self.intermediate_heading = bool(g("intermediate_heading").value)
        self.planner_timeout = float(g("planner_server_timeout").value)
        self.move_timeout = float(g("move_server_timeout").value)
        self.costmap_wait = float(g("costmap_wait").value)
        self.wp_dist_tol = float(g("wp_dist_tol").value)
        self.goal_dist_tol = float(g("goal_dist_tol").value)
        self.goal_yaw_tol = float(g("goal_yaw_tol").value)
        self.approach_dist = float(g("approach_dist").value)
        self.approach_dist_tol = float(g("approach_dist_tol").value)
        self.pose_stable_window = float(g("pose_stable_window").value)
        self.pose_stable_eps_xy = float(g("pose_stable_eps_xy").value)
        self.pose_stable_eps_yaw = float(g("pose_stable_eps_yaw").value)
        self.pose_stable_timeout = float(g("pose_stable_timeout").value)
        self.final_check_retries = int(g("final_check_retries").value)
        self.compensate_goal_to_ref = bool(g("compensate_goal_to_ref").value)
        self.goal_ref_frame = str(g("goal_ref_frame").value)
        self._ctrl_ref_offset = None   # cached (dx,dy): goal_ref_frame 在 robot_base_frame 下

        self._cb = ReentrantCallbackGroup()
        self._costmap = None
        self._costmap_lock = threading.Lock()
        self._drive_lock = threading.Lock()
        self._cancel_evt = threading.Event()
        self._active_move_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._on_costmap, 1,
            callback_group=self._cb)
        self.create_subscription(
            PoseStamped, "~/goal_pose", self._on_goal_pose, 10,
            callback_group=self._cb)

        # Latched viz so RViz sees the last plan/waypoints even if it
        # subscribes after a goal completes.
        viz_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.pub_plan = self.create_publisher(Path, "~/plan", viz_qos)
        self.pub_markers = self.create_publisher(
            MarkerArray, "~/waypoints", viz_qos)

        self._planner = ActionClient(
            self, ComputePathToPose, self.planner_action,
            callback_group=self._cb)
        self._mover = ActionClient(
            self, MoveTo, self.move_action, callback_group=self._cb)
        # position_controller 的参数服务(分层容差切换用):
        # "/position_controller/move" -> "/position_controller/set_parameters"
        from rcl_interfaces.srv import SetParameters
        self._pc_param_cli = self.create_client(
            SetParameters,
            self.move_action.rsplit("/", 1)[0] + "/set_parameters",
            callback_group=self._cb)

        self._server = ActionServer(
            self, NavigateToPose, "~/navigate",
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=self._cb)

        self.get_logger().info(
            f"waypoint_follower up: plan via '{self.planner_action}', drive via "
            f"'{self.move_action}', max_segment={self.max_segment:.1f}m, "
            f"los_threshold={self.los_threshold}")

    # ---- costmap ----
    def _on_costmap(self, msg) -> None:
        with self._costmap_lock:
            self._costmap = msg

    def _grid(self):
        with self._costmap_lock:
            return GridMapView.from_occupancy_grid(self._costmap)

    # ---- tf ----
    def _stable_map_pose(self):
        """轮询 map 位姿直到一段窗口内不再变化(定位收敛),返回收敛后位姿。

        重定位在运动中/刚停的瞬间位姿是脏的(实车:停车间隙 map 朝向自行修正
        过 42°),立刻读会把脏值当目标基准。超时未收敛则告警并用最新值。"""
        deadline = time.monotonic() + self.pose_stable_timeout
        need = max(2, int(round(self.pose_stable_window / 0.1)))
        window = []
        while time.monotonic() < deadline and not self._cancel_evt.is_set():
            try:
                window.append(self._map_pose())
            except Exception:
                window = []
            window = window[-need:]
            if len(window) >= need and pose_window_stable(
                    window, self.pose_stable_eps_xy, self.pose_stable_eps_yaw):
                return window[-1]
            time.sleep(0.1)
        self.get_logger().warn(
            f"map 位姿 {self.pose_stable_timeout:.0f}s 未收敛,用最新值(定位还在漂)")
        return self._map_pose()

    def _map_pose(self):
        tf = self.tf_buffer.lookup_transform(
            self.map_frame, self.robot_base_frame, rclpy.time.Time())
        t = tf.transform.translation
        r = tf.transform.rotation
        return (float(t.x), float(t.y),
                _yaw_from_quat(r.x, r.y, r.z, r.w))

    def _control_to_ref_offset(self):
        """(dx,dy) = goal_ref_frame 原点在 robot_base_frame(控制/测量帧)下的坐标。
        例:robot_base_frame=base_footprint、goal_ref_frame=base_link ⇒ base_link
        在 base_footprint 后方,d≈(-0.165,0)。用于把终点补偿到 base_link(见 __init__)。
        从 TF 实读 → 偏置随 URDF 变,代码无需改;查不到则退化为不补偿 (0,0) 并告警。
        结果缓存(刚体固定 TF,不变)。"""
        if (not self.compensate_goal_to_ref
                or self.goal_ref_frame == self.robot_base_frame):
            return (0.0, 0.0)
        if self._ctrl_ref_offset is not None:
            return self._ctrl_ref_offset
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.goal_ref_frame, rclpy.time.Time())
            d = (float(tf.transform.translation.x),
                 float(tf.transform.translation.y))
            self._ctrl_ref_offset = d
            self.get_logger().info(
                "[goal-comp] {}->{} offset=({:+.3f},{:+.3f})m — 终点按 {} 补偿".format(
                    self.robot_base_frame, self.goal_ref_frame,
                    d[0], d[1], self.goal_ref_frame))
            return d
        except Exception as e:
            self.get_logger().warn(
                "[goal-comp] TF {}->{} 查不到({}) — 终点不补偿".format(
                    self.robot_base_frame, self.goal_ref_frame, e))
            self._ctrl_ref_offset = (0.0, 0.0)
            return (0.0, 0.0)

    # ---- planning ----
    def _wait_grid(self):
        deadline = self.get_clock().now().nanoseconds + int(self.costmap_wait * 1e9)
        grid = self._grid()
        while grid is None and rclpy.ok():
            if self.get_clock().now().nanoseconds > deadline:
                return None
            self._cancel_evt.wait(0.1)
            grid = self._grid()
        return grid

    def _compute_path(self, goal_pose: PoseStamped):
        """Blocking call to ComputePathToPose. Returns list[(x,y)] or None."""
        if not self._planner.wait_for_server(timeout_sec=self.planner_timeout):
            self.get_logger().error("planner action server unavailable")
            return None
        req = ComputePathToPose.Goal()
        req.goal = goal_pose
        req.planner_id = self.planner_id
        req.use_start = False  # let the planner use the robot's current pose
        done = threading.Event()
        box = {}

        def _result_cb(fut):
            box["result"] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if not gh.accepted:
                box["result"] = None
                done.set()
                return
            gh.get_result_async().add_done_callback(_result_cb)

        self._planner.send_goal_async(req).add_done_callback(_goal_cb)
        done.wait()
        wrapped = box.get("result")
        if wrapped is None or wrapped.result is None:
            return None
        path = wrapped.result.path
        if not path.header.frame_id:
            path.header.frame_id = self.map_frame
        self.pub_plan.publish(path)  # viz: the dense global plan
        return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

    # ---- 分层容差:发段前切 position_controller 的容差参数 ----
    def _set_move_tols(self, dist_tol: float, yaw_tol: float) -> None:
        """服务不可用/超时只告警,沿用控制器当前容差(行为退化,不致命)。"""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        def _dp(name, val):
            return Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(val)))

        if not self._pc_param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("controller set_parameters 服务不可用,容差未切换")
            return
        req = SetParameters.Request()
        req.parameters = [_dp("pose_dist_tol", dist_tol),
                          _dp("pose_yaw_tol", yaw_tol)]
        done = threading.Event()
        box = {}

        def _on_done(fut):
            box["r"] = fut.result()
            done.set()

        self._pc_param_cli.call_async(req).add_done_callback(_on_done)
        if not done.wait(timeout=2.0):
            self.get_logger().warn("controller set_parameters 超时,容差未切换")
            return
        res = box.get("r")
        if not (res and all(r.successful for r in res.results)):
            self.get_logger().warn("controller set_parameters 被拒,容差未切换")

    # ---- driving one segment ----
    def _drive_segment(self, rx, ry, ryaw) -> bool:
        """Send one MoveTo POSE (relative). Returns True on SUCCEEDED."""
        if not self._mover.wait_for_server(timeout_sec=self.move_timeout):
            self.get_logger().error("position_controller move server unavailable")
            return False
        goal = MoveTo.Goal()
        goal.mode = MoveTo.Goal.MODE_POSE
        goal.target.x = float(rx)
        goal.target.y = float(ry)
        goal.yaw = float(ryaw)  # NaN => position-only (skip final in-place turn)
        goal.relative = True
        done = threading.Event()
        box = {}

        def _result_cb(fut):
            box["result"] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if not gh.accepted:
                box["result"] = None
                done.set()
                return
            self._active_move_handle = gh
            gh.get_result_async().add_done_callback(_result_cb)

        self._mover.send_goal_async(goal).add_done_callback(_goal_cb)
        done.wait()
        self._active_move_handle = None
        wrapped = box.get("result")
        if wrapped is None or wrapped.result is None:
            return False
        return bool(wrapped.result.success)

    # ---- the shared follow routine ----
    def _follow(self, goal_pose: PoseStamped, goal_handle=None) -> bool:
        if not self._drive_lock.acquire(blocking=False):
            self.get_logger().warn("waypoint_follower busy — goal rejected")
            if goal_handle is not None:
                goal_handle.abort()
            return False
        self._cancel_evt.clear()
        try:
            grid = self._wait_grid()
            if grid is None:
                self.get_logger().error("no global costmap — abort")
                if goal_handle is not None:
                    goal_handle.abort()
                return False

            goal_yaw = _yaw_from_quat(
                goal_pose.pose.orientation.x, goal_pose.pose.orientation.y,
                goal_pose.pose.orientation.z, goal_pose.pose.orientation.w)

            # 终点补偿到 base_link(2026-06-22):把 goal 沿最终朝向前移 -R(yaw)*d,
            # 使 base_footprint 到达补偿点、终转到 goal_yaw 后,goal_ref_frame(base_link)
            # 恰落在原 goal。补偿在规划前对整条链生效(规划/各段/进场点/末端复查
            # 全部用补偿后的点,几何一致)。仅对有终点朝向的 goal 生效——position-only
            # (yaw=NaN)无朝向定义不了"前移方向",跳过。
            if self.compensate_goal_to_ref and not math.isnan(goal_yaw):
                d0, d1 = self._control_to_ref_offset()
                if d0 or d1:
                    c, s = math.cos(goal_yaw), math.sin(goal_yaw)
                    gx0 = float(goal_pose.pose.position.x)
                    gy0 = float(goal_pose.pose.position.y)
                    goal_pose.pose.position.x = gx0 - (d0 * c - d1 * s)
                    goal_pose.pose.position.y = gy0 - (d0 * s + d1 * c)
                    self.get_logger().info(
                        "[goal-comp] goal ({:+.2f},{:+.2f}) -> {} 目标 ({:+.2f},{:+.2f}) "
                        "(沿 {:+.0f}deg 补偿,使 {} 落在原 goal)".format(
                            gx0, gy0, self.robot_base_frame,
                            goal_pose.pose.position.x, goal_pose.pose.position.y,
                            math.degrees(goal_yaw), self.goal_ref_frame))

            path_xy = self._compute_path(goal_pose)
            if not path_xy:
                self.get_logger().error("planner returned no path — abort")
                if goal_handle is not None:
                    goal_handle.abort()
                return False

            # 入场点锚定生成(2026-06-22):以"终点前入场点 A"为锚,沿路径向机器人
            # 方向按 max_segment 等距铺,短余量段落在机器人一侧;A→goal 沿最终朝向
            # 直行入位(避免末端原地大转:蹭点 + odom 角漂把 map 朝向带歪)。
            waypoints, approach_inserted = plan_waypoints(
                path_xy, grid, self.max_segment, self.los_threshold,
                self.block_unknown, goal_yaw, self.approach_dist)
            if approach_inserted:
                self.get_logger().info(
                    "[TRACE] approach wp A=({:+.2f},{:+.2f}) 锚定 goal 前 {:.2f}m,"
                    "反向按 {:.1f}m 铺到机器人(余量段在起步侧)".format(
                        waypoints[-2][0], waypoints[-2][1],
                        self.approach_dist, self.max_segment))
            elif self.approach_dist > 0.0 and not math.isnan(goal_yaw):
                self.get_logger().info(
                    "[TRACE] approach wp 不可达(A→goal 被挡),按普通路径走")
            n = len(waypoints)
            self.get_logger().info(
                f"path {len(path_xy)} pts -> {n} waypoints "
                f"(<= {self.max_segment:.1f}m, clear)")
            # [TRACE] 归因: 全局规划输出(map 系) — dense 点数/段数/各段长/各 waypoint 坐标。
            # 段数异常少、锯齿、穿墙 => 全局路径规划问题。看这行区分「全局规划 vs 单段」。
            try:
                _segs, _prev = [], path_xy[0]
                for _w in waypoints:
                    _segs.append(math.hypot(_w[0] - _prev[0], _w[1] - _prev[1]))
                    _prev = _w
                self.get_logger().info(
                    "[TRACE] plan goal=({:+.2f},{:+.2f},{:+.0f}deg) dense={} wps={} "
                    "seglen=[{}] wps_map=[{}]".format(
                        goal_pose.pose.position.x, goal_pose.pose.position.y,
                        math.degrees(goal_yaw), len(path_xy), n,
                        ",".join(f"{s:.2f}" for s in _segs),
                        " ".join(f"({w[0]:+.2f},{w[1]:+.2f})" for w in waypoints)))
            except Exception as _e:
                self.get_logger().warn(f"[TRACE] plan log failed: {_e}")
            self._publish_waypoints(path_xy[0], waypoints)  # viz

            for i, wp in enumerate(waypoints):
                if self._cancel_evt.is_set() or (
                        goal_handle is not None
                        and goal_handle.is_cancel_requested):
                    self._cancel_active_move()
                    if goal_handle is not None:
                        goal_handle.canceled()
                    self.get_logger().info("follow canceled")
                    return False

                # 段起点基准:等定位收敛再取(刚停的瞬间位姿是脏的)
                cx, cy, cyaw = self._stable_map_pose()
                rx, ry = relative_target(cx, cy, cyaw, wp[0], wp[1])
                is_last = (i == n - 1)
                if is_last and not math.isnan(goal_yaw):
                    ryaw = relative_yaw(cyaw, goal_yaw)
                elif self.intermediate_heading:
                    # face toward the segment direction
                    ryaw = math.atan2(ry, rx)
                else:
                    ryaw = float("nan")

                # [TRACE] 段开始: 机器人在 map 的当前位姿(SLAM) + 目标 waypoint(map) + 机体相对量。
                self.get_logger().info(
                    "[TRACE] seg {}/{} START map_pose=({:+.2f},{:+.2f},{:+.0f}deg) "
                    "wp_map=({:+.2f},{:+.2f}) body=({:+.2f},{:+.2f}) ryaw={}".format(
                        i + 1, n, cx, cy, math.degrees(cyaw), wp[0], wp[1],
                        rx, ry, "nan" if math.isnan(ryaw) else f"{ryaw:+.2f}"))
                # 三层容差(2026-06-10):中间段松 / 进场点较紧(保入位几何) /
                # 末段紧 + 终转
                if is_last:
                    self._set_move_tols(self.goal_dist_tol, self.goal_yaw_tol)
                elif approach_inserted and i == n - 2:
                    self._set_move_tols(self.approach_dist_tol, math.pi)
                else:
                    self._set_move_tols(self.wp_dist_tol, math.pi)
                _ok = self._drive_segment(rx, ry, ryaw)
                # [TRACE] 段结束: 比对「odom 说到位」与「map 实际残差」=> SLAM vs 控制 判据。
                #   moveto_ok=False        => 控制问题(看上面 position_controller stage: 超时/重逼近/无odom)
                #   moveto_ok=True & 残差>0.5m => SLAM 问题(map↔odom 跳变/漂移; 已打 <<< 标记)
                #   本段 END 的 map_after 与下段 START 的 map_pose 若突跳 => SLAM 跳变
                try:
                    ax, ay, ayaw = self._map_pose()
                    _resid = math.hypot(ax - wp[0], ay - wp[1])
                    _flag = ("  <<< moveto_ok 但 map 残差>0.5m: SLAM 跳变/odom-map 漂移?"
                             if (_ok and _resid > 0.5) else "")
                    self.get_logger().info(
                        "[TRACE] seg {}/{} END moveto_ok={} map_after=({:+.2f},{:+.2f},{:+.0f}deg) "
                        "residual_to_wp={:.2f}m{}".format(
                            i + 1, n, _ok, ax, ay, math.degrees(ayaw), _resid, _flag))
                except Exception as _e:
                    self.get_logger().warn(f"[TRACE] seg END log failed: {_e}")
                if not _ok:
                    self.get_logger().error(
                        f"segment {i + 1}/{n} failed — abort follow")
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False

                if goal_handle is not None:
                    fb = NavigateToPose.Feedback()
                    fb.number_of_recoveries = 0
                    try:
                        fb.distance_remaining = float(
                            sum(math.hypot(
                                waypoints[k + 1][0] - waypoints[k][0],
                                waypoints[k + 1][1] - waypoints[k][1])
                                for k in range(i, n - 1)))
                    except Exception:
                        fb.distance_remaining = 0.0
                    goal_handle.publish_feedback(fb)

            # ---- 末端闭环复查(2026-06-10) ----
            # 各段目标用的是"段开始那一刻"的定位,定位在运动后是脏的 → 终点
            # 不稳。这里停稳、等定位收敛(静止精度 ±1.5cm/0.1°),按收敛后的
            # 残差决定要不要补走一小段;复查通过才算成功 → 终点精度锚在静止
            # 定位精度上,不再受运动期漂移影响。
            gx = float(goal_pose.pose.position.x)
            gy = float(goal_pose.pose.position.y)
            for attempt in range(self.final_check_retries + 1):
                cx, cy, cyaw = self._stable_map_pose()
                resid = math.hypot(gx - cx, gy - cy)
                dyaw = (abs(relative_yaw(cyaw, goal_yaw))
                        if not math.isnan(goal_yaw) else 0.0)
                ok = resid <= self.goal_dist_tol and dyaw <= self.goal_yaw_tol
                self.get_logger().info(
                    "[TRACE] final check #{}: map=({:+.2f},{:+.2f},{:+.0f}deg) "
                    "resid={:.2f}m dyaw={:.0f}deg -> {}".format(
                        attempt + 1, cx, cy, math.degrees(cyaw), resid,
                        math.degrees(dyaw), "OK" if ok else "补走"))
                if ok:
                    break
                if attempt >= self.final_check_retries:
                    self.get_logger().error(
                        f"final check 补走 {self.final_check_retries} 轮后仍超差"
                        f"(resid={resid:.2f}m dyaw={math.degrees(dyaw):.0f}deg)"
                        f" — abort,定位侧需排查")
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False
                if self._cancel_evt.is_set() or (
                        goal_handle is not None
                        and goal_handle.is_cancel_requested):
                    self._cancel_active_move()
                    if goal_handle is not None:
                        goal_handle.canceled()
                    self.get_logger().info("follow canceled (final check)")
                    return False
                rx, ry = relative_target(cx, cy, cyaw, gx, gy)
                ryaw = (relative_yaw(cyaw, goal_yaw)
                        if not math.isnan(goal_yaw) else float("nan"))
                self._set_move_tols(self.goal_dist_tol, self.goal_yaw_tol)
                if not self._drive_segment(rx, ry, ryaw):
                    self.get_logger().error("final check 补走段失败 — abort")
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False

            if goal_handle is not None:
                goal_handle.succeed()
            self.get_logger().info("follow done: success")
            return True
        finally:
            self._drive_lock.release()

    def _publish_waypoints(self, start_xy, waypoints) -> None:
        """RViz markers: a line strip start->wp1->...->goal + a sphere per wp."""
        stamp = self.get_clock().now().to_msg()
        pts = [Point(x=float(x), y=float(y), z=0.0)
               for (x, y) in [start_xy] + list(waypoints)]

        line = Marker()
        line.header.frame_id = self.map_frame
        line.header.stamp = stamp
        line.ns = "waypoint_follower"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r = 0.1
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 1.0
        line.points = pts

        spheres = Marker()
        spheres.header.frame_id = self.map_frame
        spheres.header.stamp = stamp
        spheres.ns = "waypoint_follower"
        spheres.id = 1
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.scale.x = 0.18
        spheres.scale.y = 0.18
        spheres.scale.z = 0.18
        spheres.color.r = 1.0
        spheres.color.g = 0.6
        spheres.color.b = 0.0
        spheres.color.a = 1.0
        spheres.points = pts[1:]  # the actual waypoints (exclude robot start)

        arr = MarkerArray()
        arr.markers = [line, spheres]
        self.pub_markers.publish(arr)

    def _cancel_active_move(self) -> None:
        gh = self._active_move_handle
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass

    # ---- action callbacks ----
    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._cancel_evt.set()
        self._cancel_active_move()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        self._follow(goal_handle.request.pose, goal_handle)
        return NavigateToPose.Result()

    # ---- topic convenience ----
    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f"goal_pose -> follow target=({msg.pose.position.x:.2f},"
            f"{msg.pose.position.y:.2f}) frame={msg.header.frame_id or 'map'}")
        threading.Thread(target=self._follow, args=(msg, None),
                         daemon=True).start()


def main(argv=None):
    rclpy.init(args=argv)
    node = WaypointFollowerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
