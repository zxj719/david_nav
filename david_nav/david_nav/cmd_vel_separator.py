#!/usr/bin/env python3
"""cmd_vel mutex separator + Plan D active brake + heading guardian.

Sits at the very end of the cmd_vel chain (after david_move, before the
wheeltec serial driver). Three responsibilities:

  1. **Mutex / binary** (Plan D 2026-05-19): per incoming Twist, pick
     whichever axis dominates and zero the other. With `binary_output`,
     the surviving axis is forced to v_max_ref / ±w_max_ref so the
     smoother's intermediate ramp values can't fall into the caster
     stall zone.

  2. **Active brake** (Plan D 2026-05-20): when DWB stops asking for
     motion (input drops below threshold) but chassis is still physically
     moving (read from /odom), publish a proportional reverse cmd
     (-k_brake × actual_v) until actual velocity is near zero. This
     absorbs chassis kinetic energy — for rotation, the overshoot is
     primarily from caster-wheel drag carrying angular momentum past
     the target (~45° at ω=0.5 rad/s), NOT from PI integrator wind-up.
     Reverse cmd flips the wheel differential so caster lateral friction
     now decelerates instead of just being passive drag.

  3. **Heading guardian** (2026-05-20): if the goal direction (in robot
     frame) is more than `goal_angle_max_deg` off from the robot's forward
     axis, suppress LINEAR mode and force a brake-then-rotate cycle. This
     catches the case where DWB outputs a curved arc cmd (0.5, ω), the
     mutex collapses it to (0.5, 0), and the robot keeps driving past
     the goal because DWB's turn intent was thrown away. Release the
     suppression once the angle drops below `goal_angle_release_deg`
     (hysteresis to prevent chatter).

State machine:
  IDLE ⇄ LINEAR / ANGULAR  (input-driven, with heading guard veto)
  LINEAR  -> BRAKE_LINEAR  (input ⇒ 0 OR heading guard triggers)
  ANGULAR -> BRAKE_ANGULAR (input ⇒ 0)
  BRAKE_* -> IDLE          (actual settled, reversed, or timeout)
"""

from __future__ import annotations

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transformer


class Mode(Enum):
    IDLE = 0
    LINEAR = 1
    ANGULAR = 2
    BRAKE_LINEAR = 3
    BRAKE_ANGULAR = 4


class CmdVelSeparator(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_separator')

        # ---- params: mutex / binary ----
        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('v_max_ref', 0.20)
        self.declare_parameter('w_max_ref', 0.50)
        self.declare_parameter('hysteresis_margin', 0.10)
        self.declare_parameter('min_norm_threshold', 0.05)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('publish_hz', 2.0)
        # Sustained-low-input window: prevents single-tick DWB blips
        # from triggering input_dead → BRAKE → resume thrashing.
        self.declare_parameter('input_dead_persist_sec', 0.30)
        # Throttle periodic OUT lines to this rate. State transitions
        # are ALWAYS logged regardless (so brake/guard activations show
        # immediately).
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('binary_output', True)
        # ---- params: active brake ----
        self.declare_parameter('brake_enabled', True)
        self.declare_parameter('k_brake', 1.0)
        self.declare_parameter('max_brake_v', 0.30)
        self.declare_parameter('max_brake_w', 0.50)
        self.declare_parameter('brake_release_v', 0.05)
        self.declare_parameter('brake_release_w', 0.10)
        self.declare_parameter('brake_timeout', 2.0)
        # ---- params: heading guardian ----
        # 2026-05-20: heading guard DISABLED by default per user feedback.
        # Trust DWB's path to rotate then drive; keep the guard code so we
        # can re-enable for debugging if needed.
        self.declare_parameter('heading_guard_enabled', False)
        # Goal can arrive two ways:
        #  - /goal_pose:  RViz "2D Nav Goal" button (PoseStamped)
        #  - /plan:       planner output (last pose = goal); works for both
        #                 RViz and action-based goals (e.g.
        #                 semantic_pose_navigator.py).
        # If both have recent data, /plan wins (more current per cycle).
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('plan_max_age_sec', 2.0)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('goal_angle_max_deg', 60.0)
        self.declare_parameter('goal_angle_release_deg', 30.0)
        self.declare_parameter('goal_dist_skip', 0.50)
        # ---- params: distance-based brake (the new "1m brake") ----
        self.declare_parameter('goal_brake_enabled', True)
        self.declare_parameter('goal_brake_dist', 1.0)
        # ---- params: angle-based predictive brake ----
        # When rotating to align (ANGULAR mode) and the goal direction is
        # within `angle_brake_threshold_deg` of the robot's heading, cut
        # the rotation cmd and enter BRAKE_ANGULAR.
        #
        # Physical model (corrected 2026-05-20): the overshoot is from
        # CASTER WHEEL DRAG, not PI integrator wind-up. During in-place
        # rotation the passive caster wheels get angled sideways; the
        # robot body builds angular momentum (½·I·ω²) while caster lateral
        # friction is the only decelerating force. When cmd is finally
        # cut, the chassis coasts on inertia against caster friction —
        # overshoot ≈ ω² / (2·α_caster), roughly constant for fixed ω.
        # Pre-cutting at threshold lets the kinetic energy dissipate
        # exactly into the remaining angle.
        #
        # BRAKE_ANGULAR sends -k·ω reverse cmd which reverses the wheel
        # differential — caster friction now acts in the deceleration
        # direction (instead of just passive drag), so stopping is sharper
        # than coasting alone.
        self.declare_parameter('angle_brake_enabled', True)
        self.declare_parameter('angle_brake_threshold_deg', 45.0)
        # ---- params: silent-on-idle ----
        # When the separator is in IDLE with no recent DWB input, stop
        # publishing /cmd_vel altogether after a grace period so external
        # publishers (keyboard_teleop) can own the topic. Without this,
        # high-rate zero publishes from separator stomp on teleop cmds.
        self.declare_parameter('silent_on_idle', True)
        self.declare_parameter('idle_grace_sec', 0.5)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.v_max_ref = max(1e-6, float(self.get_parameter('v_max_ref').value))
        self.w_max_ref = max(1e-6, float(self.get_parameter('w_max_ref').value))
        self.hysteresis_margin = float(self.get_parameter('hysteresis_margin').value)
        self.min_norm_threshold = float(self.get_parameter('min_norm_threshold').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.publish_hz = max(0.1, float(self.get_parameter('publish_hz').value))
        self.input_dead_persist_sec = float(
            self.get_parameter('input_dead_persist_sec').value
        )
        self.log_period_sec = max(0.05, float(self.get_parameter('log_period_sec').value))
        self.binary_output = bool(self.get_parameter('binary_output').value)
        self.brake_enabled = bool(self.get_parameter('brake_enabled').value)
        self.k_brake = float(self.get_parameter('k_brake').value)
        self.max_brake_v = float(self.get_parameter('max_brake_v').value)
        self.max_brake_w = float(self.get_parameter('max_brake_w').value)
        self.brake_release_v = float(self.get_parameter('brake_release_v').value)
        self.brake_release_w = float(self.get_parameter('brake_release_w').value)
        self.brake_timeout = float(self.get_parameter('brake_timeout').value)
        self.heading_guard_enabled = bool(
            self.get_parameter('heading_guard_enabled').value
        )
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.plan_topic = str(self.get_parameter('plan_topic').value)
        self.plan_max_age_sec = float(self.get_parameter('plan_max_age_sec').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.goal_angle_max = math.radians(
            float(self.get_parameter('goal_angle_max_deg').value)
        )
        self.goal_angle_release = math.radians(
            float(self.get_parameter('goal_angle_release_deg').value)
        )
        self.goal_dist_skip = float(self.get_parameter('goal_dist_skip').value)
        self.goal_brake_enabled = bool(self.get_parameter('goal_brake_enabled').value)
        self.goal_brake_dist = float(self.get_parameter('goal_brake_dist').value)
        self.angle_brake_enabled = bool(self.get_parameter('angle_brake_enabled').value)
        self.angle_brake_threshold = math.radians(
            float(self.get_parameter('angle_brake_threshold_deg').value)
        )
        self.silent_on_idle = bool(self.get_parameter('silent_on_idle').value)
        self.idle_grace_sec = float(self.get_parameter('idle_grace_sec').value)

        # ---- state ----
        self.mode = Mode.IDLE
        self._last_input_stamp = self.get_clock().now()
        self._last_input_v = 0.0
        self._last_input_w = 0.0
        self._actual_v = 0.0
        self._actual_w = 0.0
        self._last_odom_stamp = self.get_clock().now() - Duration(seconds=10.0)
        self._brake_start_t = None

        # Heading guard state (sticky veto with hysteresis)
        self.linear_inhibit = False
        self._latest_goal_from_topic = None       # PoseStamped from /goal_pose
        self._latest_goal_from_plan = None        # PoseStamped from /plan last pose
        self._latest_goal_from_plan_stamp = None  # ROS Time when /plan was received
        self._latest_angle = None  # rad, signed
        self._latest_dist = None   # m
        self._guard_override_w_sign = 1  # used when forcing ANGULAR mode

        # Angular sign latch: once we enter an ANGULAR episode and pick a
        # rotation direction, stick with it across BRAKE_ANGULAR / ANGULAR
        # transitions until the episode ends (LINEAR or sustained IDLE).
        # Without this, DWB flip-flops between +ω and -ω at the symmetric
        # 180° goal point, and the separator's binary output translates
        # those flips into actual reversing wheel commands ⇒ net rotation
        # is near zero. 0 = unset / no episode active.
        self._angular_lock_sign = 0

        # input_dead temporal hysteresis: DWB sometimes emits a single
        # near-zero ω/v tick mid-rotation. Without persistence, that one
        # tick trips input_dead → BRAKE_ANGULAR → resume next tick. With
        # binary output the wasted brake/resume oscillation kills 50-70%
        # of net rotation. Require sustained dead-input for this long
        # before declaring it dead.
        self._input_dead_since = None  # timestamp when input first dipped

        # Log throttle
        self._last_log_t = self.get_clock().now() - Duration(seconds=10.0)

        # Track when we entered IDLE so we can stay silent after a grace
        # period (lets keyboard_teleop own /cmd_vel when nav2 is idle).
        self._idle_entered_t = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Goal QoS: matches RViz / bt_navigator publisher
        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(Twist, self.output_topic, qos)
        self.sub_cmd = self.create_subscription(
            Twist, self.input_topic, self._on_cmd, qos
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos
        )
        self.sub_goal = self.create_subscription(
            PoseStamped, self.goal_topic, self._on_goal, goal_qos
        )
        # /plan QoS: planner publishes Reliable + Volatile typically
        self.sub_plan = self.create_subscription(
            Path, self.plan_topic, self._on_plan, qos
        )

        self.publish_timer = self.create_timer(
            1.0 / self.publish_hz, self._publish_tick
        )

        self.get_logger().info(
            f'cmd_vel_separator up: {self.input_topic} -> {self.output_topic} '
            f'@ {self.publish_hz:.1f} Hz '
            f'(v_max_ref={self.v_max_ref:.2f}, w_max_ref={self.w_max_ref:.2f}, '
            f'binary={self.binary_output}, brake={self.brake_enabled}, '
            f'goal_brake={self.goal_brake_enabled}@{self.goal_brake_dist:.2f}m, '
            f'guard={self.heading_guard_enabled})'
        )

    # --------------- subscriptions ---------------
    def _on_cmd(self, msg: Twist) -> None:
        self._last_input_stamp = self.get_clock().now()
        self._last_input_v = float(msg.linear.x)
        self._last_input_w = float(msg.angular.z)

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_stamp = self.get_clock().now()
        self._actual_v = float(msg.twist.twist.linear.x)
        self._actual_w = float(msg.twist.twist.angular.z)

    def _on_goal(self, msg: PoseStamped) -> None:
        self._latest_goal_from_topic = msg
        self.get_logger().info(
            f'new /goal_pose received in frame "{msg.header.frame_id}": '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})'
        )

    def _on_plan(self, msg: Path) -> None:
        """Extract last pose as goal. Works for both RViz and action-based goals."""
        if not msg.poses:
            return
        last = msg.poses[-1]
        # Build a PoseStamped with the plan's frame_id (typically 'map').
        gs = PoseStamped()
        gs.header.frame_id = msg.header.frame_id
        gs.header.stamp = msg.header.stamp
        gs.pose = last.pose
        # Detect "new plan" so we can log it (don't spam every plan tick).
        is_new = (self._latest_goal_from_plan is None
                  or abs(gs.pose.position.x
                         - self._latest_goal_from_plan.pose.position.x) > 0.05
                  or abs(gs.pose.position.y
                         - self._latest_goal_from_plan.pose.position.y) > 0.05)
        self._latest_goal_from_plan = gs
        self._latest_goal_from_plan_stamp = self.get_clock().now()
        if is_new:
            self.get_logger().info(
                f'new /plan goal in "{gs.header.frame_id}": '
                f'({gs.pose.position.x:.3f}, {gs.pose.position.y:.3f}) '
                f'[{len(msg.poses)} poses]'
            )

    # --------------- helpers ---------------
    def _pick_goal_source(self):
        """Return the most current goal (plan preferred when fresh)."""
        now = self.get_clock().now()
        if self._latest_goal_from_plan is not None and self._latest_goal_from_plan_stamp is not None:
            plan_age = (now - self._latest_goal_from_plan_stamp).nanoseconds * 1e-9
            if plan_age <= self.plan_max_age_sec:
                return self._latest_goal_from_plan, 'plan'
        return self._latest_goal_from_topic, 'goal_pose'

    def _compute_goal_angle_dist(self):
        """Return (angle_rad_signed, dist_m) or (None, None) if unavailable.

        Angle is in robot frame: 0 means goal is straight ahead,
        +π/2 means goal is to robot's left, -π/2 means right.

        Note: we zero the stamp before transforming so TF uses the
        latest-available frame_id→robot_frame transform. Using the goal's
        original stamp triggers ExtrapolationException as soon as the
        TF buffer ages out that timestamp (typical buffer window is
        ~10 s), which silently breaks goal_brake right when the robot
        is closing on the goal.
        """
        goal, _ = self._pick_goal_source()
        if goal is None:
            return None, None
        try:
            # Build a stamped copy with stamp=0 → "latest available".
            from builtin_interfaces.msg import Time as TimeMsg
            stamped = PoseStamped()
            stamped.header.frame_id = goal.header.frame_id
            stamped.header.stamp = TimeMsg()  # sec=0, nanosec=0
            stamped.pose = goal.pose
            goal_in_robot = self.tf_buffer.transform(
                stamped,
                self.robot_frame,
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return None, None
        gx = goal_in_robot.pose.position.x
        gy = goal_in_robot.pose.position.y
        return math.atan2(gy, gx), math.hypot(gx, gy)

    def _brake_linear_cmd(self):
        if self._actual_v <= self.brake_release_v:
            return 0.0
        cmd = -self.k_brake * self._actual_v
        return max(cmd, -self.max_brake_v)

    def _brake_angular_cmd(self):
        if abs(self._actual_w) <= self.brake_release_w:
            return 0.0
        cmd = -self.k_brake * self._actual_w
        return max(min(cmd, self.max_brake_w), -self.max_brake_w)

    def _brake_elapsed(self):
        if self._brake_start_t is None:
            return 0.0
        return (self.get_clock().now() - self._brake_start_t).nanoseconds * 1e-9

    # --------------- main loop ---------------
    def _publish_tick(self) -> None:
        now = self.get_clock().now()
        input_age = (now - self._last_input_stamp).nanoseconds * 1e-9
        odom_age = (now - self._last_odom_stamp).nanoseconds * 1e-9
        v_in, w_in = self._last_input_v, self._last_input_w
        lin_norm = abs(v_in) / self.v_max_ref
        ang_norm = abs(w_in) / self.w_max_ref
        # Detect "input is dead" with temporal hysteresis so a single
        # near-zero tick doesn't trip BRAKE.
        instantaneous_dead = (lin_norm < self.min_norm_threshold
                              and ang_norm < self.min_norm_threshold)
        if instantaneous_dead:
            if self._input_dead_since is None:
                self._input_dead_since = now
            dead_duration = (now - self._input_dead_since).nanoseconds * 1e-9
        else:
            self._input_dead_since = None
            dead_duration = 0.0
        input_dead = (input_age > self.timeout_sec
                      or dead_duration > self.input_dead_persist_sec)

        if odom_age > 0.5:
            actual_v_safe = 0.0
            actual_w_safe = 0.0
        else:
            actual_v_safe = self._actual_v
            actual_w_safe = self._actual_w

        # ----- distance-based brake trigger (new: 1m brake) -----
        # Compute dist_to_goal first so both goal_brake and heading_guard
        # can use it. None if no goal info or no tf.
        _angle, dist_to_goal = self._compute_goal_angle_dist()
        self._latest_angle = _angle
        self._latest_dist = dist_to_goal
        # Override input_dead if within goal_brake_dist AND moving forward.
        # This is independent of what upstream is publishing — purely
        # geometric "arrival" detection.
        goal_brake_fire = False
        if (self.goal_brake_enabled and dist_to_goal is not None
                and dist_to_goal <= self.goal_brake_dist
                and actual_v_safe > self.brake_release_v):
            goal_brake_fire = True

        # Predictive ANGULAR brake: when separator is currently rotating
        # AND goal direction is within angle_brake_threshold of forward,
        # cut the rotation now and let PI overshoot finish the alignment.
        angle_brake_fire = False
        if (self.angle_brake_enabled and self._latest_angle is not None
                and abs(self._latest_angle) < self.angle_brake_threshold
                and abs(actual_w_safe) > self.brake_release_w
                and self.mode in (Mode.ANGULAR, Mode.BRAKE_ANGULAR)):
            angle_brake_fire = True

        # ----- heading guard update -----
        guard_active = False
        prev_inhibit = self.linear_inhibit
        if self.heading_guard_enabled:
            angle, dist = self._latest_angle, self._latest_dist
            if (angle is not None and dist is not None
                    and dist > self.goal_dist_skip):
                abs_ang = abs(angle)
                # Hysteresis: enter when > max, leave when < release
                if not self.linear_inhibit and abs_ang > self.goal_angle_max:
                    self.linear_inhibit = True
                    self._guard_override_w_sign = 1 if angle > 0 else -1
                elif self.linear_inhibit and abs_ang < self.goal_angle_release:
                    self.linear_inhibit = False
                if self.linear_inhibit:
                    # keep override sign fresh (in case robot swung past 0)
                    self._guard_override_w_sign = 1 if angle > 0 else -1
                guard_active = self.linear_inhibit
            else:
                # No usable goal info OR within goal_dist_skip → drop inhibit
                if self.linear_inhibit and dist is not None and dist <= self.goal_dist_skip:
                    self.linear_inhibit = False  # arriving — let the rest of the chain handle it
        if prev_inhibit != self.linear_inhibit:
            ang_deg = math.degrees(self._latest_angle) if self._latest_angle is not None else float('nan')
            dist_str = f'{self._latest_dist:.2f}' if self._latest_dist is not None else 'n/a'
            self.get_logger().info(
                f'HEADING GUARD {"ON" if self.linear_inhibit else "OFF"} '
                f'(angle={ang_deg:+.1f}°, dist={dist_str} m)'
            )

        prev_mode = self.mode

        # ----- state transitions -----
        # PRIORITY 1: goal_brake (geometric arrival, overrides input).
        # If robot is forward-moving inside goal radius, force BRAKE_LINEAR
        # regardless of what upstream is asking. Same release logic as the
        # input-driven BRAKE_LINEAR.
        # PRIORITY 1a: angle_brake — predictive rotation cut. Only triggers
        # if we were actively rotating (don't ambush teleop).
        if angle_brake_fire and prev_mode in (Mode.ANGULAR, Mode.BRAKE_ANGULAR):
            if prev_mode != Mode.BRAKE_ANGULAR:
                self._brake_start_t = now
            self.mode = Mode.BRAKE_ANGULAR
        # PRIORITY 1b: goal_brake — 1m forward cut.
        elif goal_brake_fire and prev_mode in (Mode.LINEAR, Mode.BRAKE_LINEAR):
            if prev_mode != Mode.BRAKE_LINEAR:
                self._brake_start_t = now
            self.mode = Mode.BRAKE_LINEAR
        elif input_dead:
            # Input said stop. Decide brake vs IDLE based on actual motion.
            if self.brake_enabled:
                if prev_mode in (Mode.LINEAR, Mode.BRAKE_LINEAR):
                    if actual_v_safe > self.brake_release_v:
                        if prev_mode != Mode.BRAKE_LINEAR:
                            self._brake_start_t = now
                        self.mode = Mode.BRAKE_LINEAR
                    else:
                        self.mode = Mode.IDLE
                        self._brake_start_t = None
                elif prev_mode in (Mode.ANGULAR, Mode.BRAKE_ANGULAR):
                    if abs(actual_w_safe) > self.brake_release_w:
                        if prev_mode != Mode.BRAKE_ANGULAR:
                            self._brake_start_t = now
                        self.mode = Mode.BRAKE_ANGULAR
                    else:
                        self.mode = Mode.IDLE
                        self._brake_start_t = None
                else:
                    self.mode = Mode.IDLE
            else:
                self.mode = Mode.IDLE
                self._brake_start_t = None
        else:
            # Input is active. Apply heading guard veto first.
            if guard_active:
                # Linear is forbidden — either brake, or force ANGULAR.
                if prev_mode == Mode.LINEAR:
                    # Was driving forward — brake first.
                    if actual_v_safe > self.brake_release_v and self.brake_enabled:
                        if prev_mode != Mode.BRAKE_LINEAR:
                            self._brake_start_t = now
                        self.mode = Mode.BRAKE_LINEAR
                    else:
                        self.mode = Mode.ANGULAR
                        self._brake_start_t = None
                elif prev_mode == Mode.BRAKE_LINEAR:
                    # Keep braking until actual_v drops, then go ANGULAR.
                    if actual_v_safe > self.brake_release_v:
                        self.mode = Mode.BRAKE_LINEAR
                    else:
                        self.mode = Mode.ANGULAR
                        self._brake_start_t = None
                elif prev_mode in (Mode.IDLE, Mode.BRAKE_ANGULAR, Mode.ANGULAR):
                    self.mode = Mode.ANGULAR
                    self._brake_start_t = None
            else:
                # Normal mutex selection.
                self._brake_start_t = None
                if prev_mode in (Mode.IDLE, Mode.BRAKE_LINEAR, Mode.BRAKE_ANGULAR):
                    self.mode = Mode.ANGULAR if ang_norm >= lin_norm else Mode.LINEAR
                elif prev_mode == Mode.LINEAR:
                    if ang_norm > lin_norm + self.hysteresis_margin:
                        self.mode = Mode.ANGULAR
                elif prev_mode == Mode.ANGULAR:
                    if lin_norm > ang_norm + self.hysteresis_margin:
                        self.mode = Mode.LINEAR

        # Brake timeout / release check
        if self.mode == Mode.BRAKE_LINEAR:
            if (actual_v_safe < self.brake_release_v
                    or actual_v_safe < 0.0
                    or self._brake_elapsed() > self.brake_timeout):
                self.mode = (Mode.ANGULAR if guard_active else Mode.IDLE)
                self._brake_start_t = None
        elif self.mode == Mode.BRAKE_ANGULAR:
            if (abs(actual_w_safe) < self.brake_release_w
                    or self._brake_elapsed() > self.brake_timeout):
                self.mode = Mode.IDLE
                self._brake_start_t = None

        # ----- angular sign latch update -----
        # Initialise lock when entering ANGULAR mode; clear when leaving
        # any rotation episode (LINEAR or sustained IDLE).
        if self.mode == Mode.ANGULAR and prev_mode not in (Mode.ANGULAR, Mode.BRAKE_ANGULAR):
            # New rotation episode. Pick sign from DWB's current cmd
            # (if non-zero) else from heading guard's computed direction
            # (if active) else default +1.
            if w_in != 0.0:
                self._angular_lock_sign = 1 if w_in > 0.0 else -1
            elif guard_active:
                self._angular_lock_sign = self._guard_override_w_sign
            else:
                # Pick toward shortest-rotation direction if we know the angle.
                if self._latest_angle is not None and self._latest_angle != 0.0:
                    self._angular_lock_sign = 1 if self._latest_angle > 0.0 else -1
                else:
                    self._angular_lock_sign = 1
        if self.mode == Mode.LINEAR or self.mode == Mode.IDLE:
            self._angular_lock_sign = 0

        # ----- compute output -----
        out = Twist()
        if self.mode == Mode.LINEAR:
            out.linear.x = self.v_max_ref if self.binary_output else v_in
        elif self.mode == Mode.ANGULAR:
            if guard_active:
                # Heading guard: use computed direction toward goal, NOT input.
                out.angular.z = self._guard_override_w_sign * self.w_max_ref
            elif self.binary_output:
                # Sign-latch: use the locked sign for the whole rotation
                # episode. DWB's mid-rotation flips do NOT propagate; this
                # prevents the bunny-hop CW/CCW oscillation that wastes ~90%
                # of rotation effort when goal is near the 180° symmetry point.
                lock = self._angular_lock_sign
                if lock == 0:
                    # Fallback (shouldn't happen): use DWB sign
                    lock = 1 if (w_in >= 0.0) else -1
                out.angular.z = lock * self.w_max_ref
            else:
                out.angular.z = w_in
        elif self.mode == Mode.BRAKE_LINEAR:
            out.linear.x = self._brake_linear_cmd()
        elif self.mode == Mode.BRAKE_ANGULAR:
            out.angular.z = self._brake_angular_cmd()

        # ----- silent-on-idle: stop hammering /cmd_vel when truly idle, so
        # keyboard_teleop (or any other ad-hoc publisher) can own the
        # topic. We still publish for a short grace period after entering
        # IDLE to make sure the chassis sees a stop command. -----
        if self.mode == Mode.IDLE:
            if prev_mode != Mode.IDLE or self._idle_entered_t is None:
                self._idle_entered_t = now
            idle_age = (now - self._idle_entered_t).nanoseconds * 1e-9
        else:
            self._idle_entered_t = None
            idle_age = 0.0

        silent = (self.silent_on_idle
                  and self.mode == Mode.IDLE
                  and input_age > self.timeout_sec
                  and idle_age > self.idle_grace_sec
                  and not goal_brake_fire)

        if silent:
            # Do NOT publish anything — let external publishers own /cmd_vel.
            # Throttled info so we know the separator is in silent mode.
            log_age = (now - self._last_log_t).nanoseconds * 1e-9
            if log_age >= self.log_period_sec:
                self._last_log_t = now
                self.get_logger().info(
                    f'IDLE silent (input_age={input_age*1000:.0f}ms, '
                    f'idle_age={idle_age:.1f}s) — yielding /cmd_vel'
                )
            return

        self.pub.publish(out)

        # ----- log -----
        # State transitions ALWAYS print (so brake / guard fire is visible).
        if self.mode != prev_mode:
            ang_deg = (math.degrees(self._latest_angle)
                       if self._latest_angle is not None else float('nan'))
            self.get_logger().info(
                f'STATE {prev_mode.name} → {self.mode.name} '
                f'(guard={"ON" if guard_active else "off"}, '
                f'angle2goal={ang_deg:+.1f}°, '
                f'in: v={v_in:+0.3f} w={w_in:+0.3f}, '
                f'act: v={self._actual_v:+0.3f} w={self._actual_w:+0.3f})'
            )
        # Periodic OUT line throttled to log_period_sec.
        log_age = (now - self._last_log_t).nanoseconds * 1e-9
        if log_age >= self.log_period_sec:
            self._last_log_t = now
            ang_deg = (math.degrees(self._latest_angle)
                       if self._latest_angle is not None else float('nan'))
            dist_str = (f'{self._latest_dist:.2f}'
                        if self._latest_dist is not None else 'n/a')
            self.get_logger().info(
                f'OUT mode={self.mode.name:<14} '
                f'v={out.linear.x:+0.3f} w={out.angular.z:+0.3f}  '
                f'guard={"ON" if guard_active else "off"} '
                f'a2g={ang_deg:+5.1f}° d2g={dist_str}m '
                f'(in: v={v_in:+0.3f} w={w_in:+0.3f}, '
                f'act: v={self._actual_v:+0.3f} w={self._actual_w:+0.3f})'
            )


def main(argv=None):
    rclpy.init(args=argv)
    node = CmdVelSeparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
