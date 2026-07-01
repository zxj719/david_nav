#!/usr/bin/env python3
"""Pure control-math core for the David position-loop controller.

NO rclpy / ROS imports live here on purpose: every value in / value out, so
the whole closed-loop algorithm (PID, trapezoid limiting, distance projection,
turn-drive-turn POSE state machine, arrival latch) is unit-testable without a
running node — mirroring the existing ``navigation_executor_core`` /
``scan_priority_geometry`` split in this package.

The rclpy node (``position_controller.py``) is a thin shell: it pulls pose +
dt out of /odom, calls :meth:`PositionController.update`, and forwards the
returned (v, w) to /cmd_vel_position.

Algorithm provenance: WHEELTEC PID 学习套件例程 6 (位置式 PID, P+D 主导, 小 Ki +
积分限幅, 输出限速). 区别: 套件闭单电机转 N 圈; 这里闭整车位姿 (距离, yaw) → (v, ω).
See SPEC 位置环实现方案 §3.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# --------------------------------------------------------------------------- #
# small math helpers (free functions so tests can hit them directly)
# --------------------------------------------------------------------------- #
def wrap_to_pi(angle: float) -> float:
    """Wrap any angle into (-pi, pi]. atan2(sin,cos) — never hand-roll this."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def rate_limit(target: float, previous: float, max_delta: float) -> float:
    """Slew-limit ``target`` so it moves at most ``max_delta`` from ``previous``.

    ``max_delta`` should already be (accel_limit * dt) and non-negative.
    """
    if max_delta <= 0.0:
        return target
    delta = clamp(target - previous, -max_delta, max_delta)
    return previous + delta


def project_travelled(x: float, y: float, x0: float, y0: float, yaw0: float) -> float:
    """Signed distance travelled along the *start* heading (yaw0).

    Forward progress is positive; sideways drift does not count. This is the
    DISTANCE-mode odometer: err_d = distance - project_travelled(...).
    """
    return (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)


# --------------------------------------------------------------------------- #
# configuration dataclasses (filled from ros params by the node)
# --------------------------------------------------------------------------- #
@dataclass
class PidGains:
    kp: float
    ki: float
    kd: float
    i_max: float


@dataclass
class Limits:
    v_max: float = 0.25
    w_max: float = 0.8
    a_max: float = 0.5
    alpha_max: float = 1.5
    # Angular DECEL limit, separate from alpha_max (the accel limit), mirroring
    # velocity_smoother's max_accel/max_decel split. The rotate_lead "cut to zero"
    # (_turn_toward) routes its 0-command through the slew limiter; with a single
    # symmetric alpha_max the command coasts down over ~w/alpha_max seconds,
    # actively COMMANDING ~½·w·(w/alpha_max) rad of extra rotation AFTER the cut —
    # the controller-induced bulk of the rotation overshoot (2026-06-30 diagnosis).
    # A larger decel limit lets the cut actually stop fast (less over-rotation)
    # while keeping the turn START gentle (alpha_max) for upper-body stability.
    # 0 = fall back to alpha_max (symmetric, original behaviour). Angular only;
    # linear stays symmetric (a_max) — straight-line braking uses approach_decel.
    alpha_decel_max: float = 0.0
    # Angular APPROACH-deceleration (2026-07-01). The angular twin of
    # approach_decel: near the target cap |w| by the braking-speed profile
    # w <= sqrt(2*alpha_approach*err_eff) so the heavy chassis arrives at ~floor
    # speed instead of full speed. On this plant the terminal overshoot is
    # momentum-dominated (it rotates ~1.6 rad/s at cmd 0.75 and coasts ~49° on
    # friction regardless of the command ramp), so the ONLY lever that shrinks it
    # is arriving slower. 0 = off (no cap). Real-car value in the yaml; tune on car.
    alpha_approach: float = 0.0
    # Approach speed cap (2026-06-09): near the target limit v to the
    # braking-distance speed  v <= sqrt(2*approach_decel*rho_eff)  so the heavy
    # chassis decelerates INTO the goal instead of coasting past (inertial
    # overshoot). Smaller = start slowing earlier / gentler = less overshoot.
    # Default ≈ a_max (mild / near no-op); real-car value lives in the yaml.
    approach_decel: float = 0.5  # m/s^2
    # Deadzone floors — needed on the real chassis: /cmd_vel_position bypasses
    # david_move's lift and firmware break-away FF is NOT done. Defaults here are
    # 0 (sim/bench safe); the real-car values live in position_controller.yaml
    # (report break-away ≈0.05 / ≈0.18). See README "与底盘死区".
    min_cmd_v: float = 0.0   # linear deadzone floor (治标)
    min_cmd_w: float = 0.0   # angular deadzone floor (治标)


@dataclass
class Tolerances:
    dist_tol: float = 0.02         # DISTANCE-mode / straight-line arrival (stays tight)
    pose_dist_tol: float = 0.05    # POSE position arrival — 5cm (caster-limited plant)
    yaw_tol: float = 0.02          # ROTATE-mode arrival (stays tight = precise turn)
    pose_yaw_tol: float = 0.35     # POSE final-heading arrival — ~20° (loose = fast)
    pose_align_tol: float = 0.10   # rad, "aligned enough" gate before DRIVE
    # Rotation-overshoot feedforward: cut the in-place turn this early and let
    # caster momentum coast the rest (instead of PID-overshooting then settling).
    rotate_lead: float = 0.15      # rad (~9°); tune up toward ~20° on the car
    # Drive-drift pre-aim: a freshly in-place-turned chassis SWINGS as the casters
    # re-orient when forward motion starts. We do NOT re-align mid-drive (that
    # re-swivels the casters → infinite loop); instead ALIGN aims OFF by this much
    # so the post-swing straight drive ends on the point. Signed, EMPIRICAL —
    # calibrate magnitude + sign on the car (0 = off).
    drive_drift_lead: float = 0.0  # rad
    # If ρ rebounds this far past its closest approach, we sailed past the point
    # (mis-aimed) → stop and re-approach rather than circling.
    drive_overshoot_margin: float = 0.05  # m
    pose_max_cycles: int = 4
    arrival_cycles: int = 10
    v_stop: float = 0.01           # |measured v| below this counts as stopped
    w_stop: float = 0.05           # |measured w| below this counts as stopped


class Mode(Enum):
    DISTANCE = 0
    ROTATE = 1
    POSE = 2


class PoseStage(Enum):
    ALIGN = 0       # turn in place to face the target point (pure omega)
    DRIVE = 1       # drive straight to the point (pure v, NO omega — caster safe)
    FINAL_TURN = 2  # turn in place to the commanded final heading (pure omega)


@dataclass
class Goal:
    """A goal already resolved into the odom frame by the node.

    For POSE the node converts a relative request into an absolute odom point
    + heading once, at accept time, and never re-derives the start (SPEC §5).
    """
    mode: Mode
    distance: float = 0.0              # DISTANCE: signed metres along heading
    yaw: float = 0.0                   # ROTATE: relative rad; POSE: final heading (odom)
    target_x: float = 0.0              # POSE: odom target point
    target_y: float = 0.0
    has_final_heading: bool = True     # POSE: do the final-turn stage?


@dataclass
class ControlOutput:
    v: float = 0.0
    w: float = 0.0
    done: bool = False
    success: bool = False
    error_remaining: float = 0.0   # metres (DISTANCE/POSE-drive) or rad (ROTATE/turn)
    stage: str = ""                # human-readable phase for feedback/logging


# --------------------------------------------------------------------------- #
# position-form PID (例程 6 结构), one instance per scalar error channel
# --------------------------------------------------------------------------- #
class Pid:
    def __init__(self, gains: PidGains) -> None:
        self.g = gains
        self.integral = 0.0
        self.prev_err: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_err = None

    def step(self, err: float, dt: float) -> float:
        # integral with anti-windup clamp (必须)
        self.integral += err * dt
        self.integral = clamp(self.integral, -self.g.i_max, self.g.i_max)
        # derivative on error; first sample has no history → 0 (no setpoint kick)
        if self.prev_err is None or dt <= 0.0:
            deriv = 0.0
        else:
            deriv = (err - self.prev_err) / dt
        u = self.g.kp * err + self.g.ki * self.integral + self.g.kd * deriv
        self.prev_err = err
        return u


# --------------------------------------------------------------------------- #
# the controller
# --------------------------------------------------------------------------- #
class PositionController:
    """Stateful but pure (no I/O) closed-loop body-pose controller.

    Lifecycle: configure(...) once, then begin(goal, x0, y0, yaw0) per goal,
    then update(x, y, yaw, dt, meas_v, meas_w) every control tick until
    ``ControlOutput.done``.
    """

    def __init__(
        self,
        limits: Limits,
        dist_gains: PidGains,
        yaw_gains: PidGains,
        tol: Tolerances,
    ) -> None:
        # floor 在 clamp/slew 之后施加,floor > max 会让输出恒超上限 ——
        # 矛盾配置,启动即报错(2026-06-10 实测最坏角死区 0.30 == 当时 w_max=0.3)。
        if limits.min_cmd_v > limits.v_max or limits.min_cmd_w > limits.w_max:
            raise ValueError(
                f"deadzone floor exceeds speed cap: min_cmd_v={limits.min_cmd_v}"
                f"/v_max={limits.v_max}, min_cmd_w={limits.min_cmd_w}"
                f"/w_max={limits.w_max}")
        self.limits = limits
        self.tol = tol
        self.dist_pid = Pid(dist_gains)
        self.yaw_pid = Pid(yaw_gains)

        # per-goal state (set in begin)
        self.goal: Optional[Goal] = None
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw0 = 0.0
        self.v_prev = 0.0
        self.w_prev = 0.0
        self._arrival_count = 0
        self._pose_stage = PoseStage.ALIGN

    # ------------------------------------------------------------------ #
    def begin(self, goal: Goal, x0: float, y0: float, yaw0: float) -> None:
        self.goal = goal
        self.x0 = x0
        self.y0 = y0
        self.yaw0 = yaw0
        self.v_prev = 0.0
        self.w_prev = 0.0
        self._arrival_count = 0
        self._pose_stage = PoseStage.ALIGN
        self._pose_cycles = 0
        self._rho_min = float("inf")   # closest approach this DRIVE (miss detect)
        # Caster-state model: the drive-start swing direction is set by the
        # ALIGN turn direction. Captured once per ALIGN entry; the drift lead's
        # sign follows it so it works for turns either way (not just the test).
        self._align_turn_dir = 1.0
        self._capture_turn_dir = True
        self.dist_pid.reset()
        self.yaw_pid.reset()

    # ------------------------------------------------------------------ #
    @staticmethod
    def _floor(out: float, target: float, floor: float, active: bool) -> float:
        """Dead-band lift, applied AFTER the slew limit (2026-06-10).

        死区 (0, floor) 是非物理区间:指令在里面底盘纹丝不动,然后 stick-slip
        暴冲。旧顺序(floor 再 rate_limit)让起步斜坡 0.01/0.02… 在死区里慢爬。
        现在:只要还想动(active 且 target≠0),斜坡值落进死区就直接抬到
        ±floor;符号跟 *target*(不是斜坡值),反向时一拍跨过死区,不会锁死在
        旧符号的 +floor 上。跳过死区不产生真实冲击(底盘在死区里本来就不动),
        与 separator binary_output「端点踢穿静摩擦」同思想。
        """
        if floor <= 0.0 or not active or target == 0.0:
            return out
        if abs(out) < floor:
            return math.copysign(floor, target)
        return out

    def _shape(self, v_cmd: float, w_cmd: float, dt: float,
               v_active: bool, w_active: bool) -> tuple:
        """clamp → trapezoid slew limit → deadzone floor. Updates v/w_prev."""
        lim = self.limits
        v_cmd = clamp(v_cmd, -lim.v_max, lim.v_max)
        w_cmd = clamp(w_cmd, -lim.w_max, lim.w_max)
        v_out = rate_limit(v_cmd, self.v_prev, lim.a_max * dt)
        # Angular slew is asymmetric: gentle accel (alpha_max) but optionally fast
        # decel (alpha_decel_max) so the rotate_lead cut actually stops instead of
        # coasting the command down over ~1s. "Decelerating" = the command
        # magnitude is shrinking toward zero (|target| < |prev|) — exactly the
        # lead-cut (target 0) and any slow-down; speed-ups keep the gentle limit.
        alpha_dec = lim.alpha_decel_max if lim.alpha_decel_max > 0.0 else lim.alpha_max
        w_alpha = alpha_dec if abs(w_cmd) < abs(self.w_prev) else lim.alpha_max
        w_out = rate_limit(w_cmd, self.w_prev, w_alpha * dt)
        v_out = self._floor(v_out, v_cmd, lim.min_cmd_v, v_active)
        w_out = self._floor(w_out, w_cmd, lim.min_cmd_w, w_active)
        self.v_prev = v_out
        self.w_prev = w_out
        return v_out, w_out

    def _latch(self, err: float, tol: float, stopped: bool) -> bool:
        """True once |err|<tol for arrival_cycles consecutive ticks while stopped.

        Resets the moment the error leaves the band — guards against a
        single-tick overshoot reading as 'arrived' (SPEC §3.4).
        """
        if abs(err) < tol and stopped:
            self._arrival_count += 1
        else:
            self._arrival_count = 0
        return self._arrival_count >= self.tol.arrival_cycles

    # ------------------------------------------------------------------ #
    def update(self, x: float, y: float, yaw: float, dt: float,
               meas_v: float = 0.0, meas_w: float = 0.0) -> ControlOutput:
        if self.goal is None or dt <= 0.0:
            return ControlOutput(v=0.0, w=0.0, done=False, stage="idle")
        if self.goal.mode is Mode.DISTANCE:
            return self._update_distance(x, y, yaw, dt, meas_v, meas_w)
        if self.goal.mode is Mode.ROTATE:
            return self._update_rotate(yaw, dt, meas_w)
        return self._update_pose(x, y, yaw, dt, meas_v, meas_w)

    # ---- DISTANCE -------------------------------------------------- #
    def _update_distance(self, x, y, yaw, dt, meas_v, meas_w) -> ControlOutput:
        travelled = project_travelled(x, y, self.x0, self.y0, self.yaw0)
        err_d = self.goal.distance - travelled
        err_yaw = wrap_to_pi(self.yaw0 - yaw)   # hold the start heading

        stopped = abs(meas_v) < self.tol.v_stop
        if self._latch(err_d, self.tol.dist_tol, stopped):
            self.v_prev = 0.0
            self.w_prev = 0.0
            return ControlOutput(v=0.0, w=0.0, done=True, success=True,
                                 error_remaining=err_d, stage="arrived")

        v_cmd = self.dist_pid.step(err_d, dt)
        w_cmd = self.yaw_pid.step(err_yaw, dt)
        v_active = abs(err_d) >= self.tol.dist_tol
        v_cmd, w_cmd = self._shape(v_cmd, w_cmd, dt, v_active, False)
        return ControlOutput(v=v_cmd, w=w_cmd, error_remaining=err_d,
                             stage="distance")

    # ---- shared lead-angle turn command ---------------------------- #
    def _turn_toward(self, err: float, dt: float, meas_w: float,
                     tol: float) -> float:
        """Pure-omega command toward signed angular error `err`.

        Lead-angle feedforward (caster rotation-overshoot comp): once within
        rotate_lead of the target AND still spinning, cut the command to zero and
        let momentum coast the rest. If it settles short of `tol`, a gentle PID
        nudge resumes. Outside the lead band it's a normal floored PID turn.
        """
        if abs(err) <= self.tol.rotate_lead and abs(meas_w) >= self.tol.w_stop:
            _, w = self._shape(0.0, 0.0, dt, False, False)   # coast / ramp to 0
            return w
        w_active = abs(err) > tol
        w_pid = self.yaw_pid.step(err, dt) if w_active else 0.0
        # Angular approach-decel: cap |w| by the braking-speed profile toward the
        # target so the chassis decelerates INTO the goal (arrives ~floor speed →
        # small momentum coast) instead of hitting the lead-cut at full speed. The
        # deadzone floor in _shape still guarantees the minimum break-away speed,
        # so this only bites in the mid-approach, not the last degree.
        if self.limits.alpha_approach > 0.0:
            w_cap = math.sqrt(2.0 * self.limits.alpha_approach * abs(err))
            w_pid = clamp(w_pid, -w_cap, w_cap)
        _, w = self._shape(0.0, w_pid, dt, False, w_active)
        return w

    # ---- ROTATE ---------------------------------------------------- #
    def _update_rotate(self, yaw, dt, meas_w) -> ControlOutput:
        yaw_target = self.yaw0 + self.goal.yaw
        err_yaw = wrap_to_pi(yaw_target - yaw)

        stopped = abs(meas_w) < self.tol.w_stop
        if self._latch(err_yaw, self.tol.yaw_tol, stopped):
            self.v_prev = 0.0
            self.w_prev = 0.0
            return ControlOutput(v=0.0, w=0.0, done=True, success=True,
                                 error_remaining=err_yaw, stage="arrived")

        w_cmd = self._turn_toward(err_yaw, dt, meas_w, self.tol.yaw_tol)
        return ControlOutput(v=0.0, w=w_cmd, error_remaining=err_yaw,
                             stage="rotate")

    # ---- POSE (turn-drive-turn) ------------------------------------ #
    def _enter_stage(self, stage: PoseStage) -> None:
        if stage is not self._pose_stage:
            self._pose_stage = stage
            self._arrival_count = 0
            self.dist_pid.reset()
            self.yaw_pid.reset()
            if stage is PoseStage.DRIVE:
                self._rho_min = float("inf")
            if stage is PoseStage.ALIGN:
                self._capture_turn_dir = True   # re-capture turn dir each ALIGN

    def _update_pose(self, x, y, yaw, dt, meas_v, meas_w) -> ControlOutput:
        """Caster-aware turn-drive-turn. INVARIANT: every tick is pure v OR pure
        omega, never both — arcing jams the passive casters (the reason
        cmd_vel_separator mutexes line/angular).

        Caster handling (per 2026-06-09 real-car diagnosis):
          * ROTATION overshoots from caster momentum → rotate_lead feedforward
            cuts the turn early and coasts.
          * A freshly turned chassis SWINGS when forward motion starts (casters
            re-orient). We do NOT re-align mid-drive — an in-place re-align just
            re-swivels the casters and the next drive swings again, looping
            forever (observed: 60s churn). Instead ALIGN aims OFF by
            drive_drift_lead so the post-swing straight drive lands on the point,
            and we drive straight through. If we sail past (mis-aim), ρ rebounds
            → re-approach (bounded by pose_max_cycles).
        """
        dx = self.goal.target_x - x
        dy = self.goal.target_y - y
        rho = math.hypot(dx, dy)
        settled_w = abs(meas_w) < self.tol.w_stop
        settled_v = abs(meas_v) < self.tol.v_stop

        # ---- ALIGN: pure in-place turn, pre-aimed against the drive swing ----
        if self._pose_stage is PoseStage.ALIGN:
            if rho <= self.tol.pose_dist_tol:   # already on the point (degenerate)
                if self.goal.has_final_heading:
                    self._enter_stage(PoseStage.FINAL_TURN)
                    return ControlOutput(v=0.0, w=0.0, error_remaining=rho,
                                         stage="pose_align_skip")
                self.v_prev = self.w_prev = 0.0
                return ControlOutput(v=0.0, w=0.0, done=True, success=True,
                                     error_remaining=rho, stage="arrived")
            # aim OFF the true bearing by drive_drift_lead so the drive-start
            # caster swing brings us back onto the line to the point. The SIGN
            # follows the ALIGN turn direction (the turn sets the casters): a CCW
            # turn swings the drive one way, CW the other.
            bearing = math.atan2(dy, dx)
            beta = wrap_to_pi(bearing - yaw)   # error to the TRUE bearing (no lead)
            if self._capture_turn_dir:
                self._align_turn_dir = 1.0 if beta >= 0.0 else -1.0
                self._capture_turn_dir = False
            # drive_drift_lead 只在「还朝 bearing 转、且未到位」时施加。
            # 已到位(|beta|<=align_tol)或已超调(转过 bearing → beta 与转向反号)时
            # 丢弃 lead，避免再多转一个 lead 角(2026-06-09 用户要求)。
            overshot = (beta * self._align_turn_dir) < 0.0
            in_tol = abs(beta) <= self.tol.pose_align_tol
            lead = 0.0 if (overshot or in_tol) else self.tol.drive_drift_lead
            aim = wrap_to_pi(bearing - lead * self._align_turn_dir)
            alpha = wrap_to_pi(aim - yaw)
            if abs(alpha) <= self.tol.pose_align_tol and settled_w:
                self._enter_stage(PoseStage.DRIVE)
                self.w_prev = 0.0
                return ControlOutput(v=0.0, w=0.0, error_remaining=alpha,
                                     stage="pose_align_settled")
            w_cmd = self._turn_toward(alpha, dt, meas_w, self.tol.pose_align_tol)
            return ControlOutput(v=0.0, w=w_cmd, error_remaining=alpha,
                                 stage="pose_align")

        # ---- DRIVE: pure straight, NO mid-drive re-align (caster-safe) ----
        if self._pose_stage is PoseStage.DRIVE:
            arrived = rho <= self.tol.pose_dist_tol and settled_v
            # sailed past closest approach (mis-aimed) → stop, re-approach
            missed = (rho > self._rho_min + self.tol.drive_overshoot_margin)
            if arrived or (missed and not (self._pose_cycles < self.tol.pose_max_cycles)):
                if self.goal.has_final_heading:
                    self._enter_stage(PoseStage.FINAL_TURN)
                    self.v_prev = 0.0
                    return ControlOutput(v=0.0, w=0.0, error_remaining=rho,
                                         stage="pose_drive_done")
                self.v_prev = self.w_prev = 0.0
                return ControlOutput(v=0.0, w=0.0, done=True,
                                     success=rho <= 2.0 * self.tol.pose_dist_tol,
                                     error_remaining=rho, stage="arrived")
            if missed:
                if not settled_v:   # brake to a stop before turning
                    v_cmd, _ = self._shape(0.0, 0.0, dt, False, False)
                    return ControlOutput(v=v_cmd, w=0.0, error_remaining=rho,
                                         stage="pose_drive_brake")
                self._pose_cycles += 1
                self._enter_stage(PoseStage.ALIGN)
                return ControlOutput(v=0.0, w=0.0, error_remaining=rho,
                                     stage="pose_reapproach")
            self._rho_min = min(self._rho_min, rho)
            v_active = rho > self.tol.pose_dist_tol
            v_cmd = self.dist_pid.step(rho, dt) if v_active else 0.0
            # 接近目标按可刹停距离限速,减少惯性超调(2026-06-09 用户要求):
            # v <= sqrt(2*approach_decel*(rho - pose_dist_tol))，rho→tol 时 v→0,
            # 所以末端是减速逼近而非满速切断后冲过去。approach_decel 越小越早减速。
            rho_eff = max(rho - self.tol.pose_dist_tol, 0.0)
            v_brake_cap = math.sqrt(2.0 * self.limits.approach_decel * rho_eff)
            v_cmd = clamp(v_cmd, -v_brake_cap, v_brake_cap)
            v_cmd, _ = self._shape(v_cmd, 0.0, dt, v_active, False)
            return ControlOutput(v=v_cmd, w=0.0, error_remaining=rho,
                                 stage="pose_drive")

        # ---- FINAL_TURN: pure in-place turn to the final heading ----
        err_yaw = wrap_to_pi(self.goal.yaw - yaw)
        if abs(err_yaw) <= self.tol.pose_yaw_tol and settled_w:
            # the in-place turn scrubs the point (casters swivel) → re-check
            if rho > self.tol.pose_dist_tol and self._pose_cycles < self.tol.pose_max_cycles:
                self._pose_cycles += 1
                self._enter_stage(PoseStage.ALIGN)
                return ControlOutput(v=0.0, w=0.0, error_remaining=rho,
                                     stage="pose_recheck")
            self.v_prev = self.w_prev = 0.0
            success = rho <= 2.0 * self.tol.pose_dist_tol
            return ControlOutput(v=0.0, w=0.0, done=True, success=success,
                                 error_remaining=err_yaw, stage="arrived")
        w_cmd = self._turn_toward(err_yaw, dt, meas_w, self.tol.pose_yaw_tol)
        return ControlOutput(v=0.0, w=w_cmd, error_remaining=err_yaw,
                             stage="pose_final_turn")
