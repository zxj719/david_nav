"""Unit tests for position_control_core (pure, no ROS).

A tiny ideal differential-drive integrator stands in for the chassis so we can
verify the closed loop converges and respects the SPEC §7 acceptance bounds.
"""

import math

from david_nav.position_control_core import (
    Goal,
    Limits,
    Mode,
    PidGains,
    PositionController,
    Tolerances,
    clamp,
    project_travelled,
    rate_limit,
    wrap_to_pi,
)


# ---------- free-function math ----------
def test_wrap_to_pi_basic():
    # result must always land in (-pi, pi]; ±pi are the same angle, so compare
    # via the wrapped difference rather than the raw representative.
    def _same(a, b):
        return abs(wrap_to_pi(a - b)) < 1e-9

    assert math.isclose(wrap_to_pi(0.0), 0.0)
    assert _same(wrap_to_pi(math.pi), math.pi)
    assert _same(wrap_to_pi(3.0 * math.pi), math.pi)
    assert _same(wrap_to_pi(-3.0 * math.pi), math.pi)
    assert math.isclose(wrap_to_pi(1.5 * math.pi), -0.5 * math.pi, abs_tol=1e-9)
    assert -math.pi < wrap_to_pi(123.4) <= math.pi


def test_clamp():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.3, -1.0, 1.0) == 0.3


def test_rate_limit_caps_step():
    assert rate_limit(1.0, 0.0, 0.1) == 0.1
    assert rate_limit(-1.0, 0.0, 0.1) == -0.1
    assert rate_limit(0.05, 0.0, 0.1) == 0.05
    assert rate_limit(1.0, 0.0, 0.0) == 1.0  # 0 max_delta => passthrough


def test_project_travelled_along_heading():
    # heading 0: travelled = dx
    assert math.isclose(project_travelled(2.0, 5.0, 0.0, 0.0, 0.0), 2.0)
    # heading +90deg: forward is +y
    assert math.isclose(
        project_travelled(0.0, 3.0, 0.0, 0.0, math.pi / 2), 3.0, abs_tol=1e-9)
    # sideways drift doesn't count
    assert math.isclose(
        project_travelled(0.0, 9.0, 0.0, 0.0, 0.0), 0.0, abs_tol=1e-9)


# ---------- helpers ----------
def _make_controller(**over):
    limits = Limits(v_max=0.25, w_max=0.8, a_max=0.5, alpha_max=1.5,
                    **over.get("limits", {}))
    dist = PidGains(1.2, 0.0, 0.1, 0.3)
    yaw = PidGains(2.0, 0.0, 0.15, 0.5)
    tol = Tolerances(dist_tol=0.02, pose_dist_tol=0.02, yaw_tol=0.02, pose_yaw_tol=0.02,
                     pose_align_tol=0.05, arrival_cycles=10,
                     v_stop=0.01, w_stop=0.05)
    return PositionController(limits, dist, yaw, tol)


def _simulate(ctrl, x, y, yaw, dt=0.02, max_steps=5000):
    """Run the loop against an ideal unicycle plant; return final state + out."""
    last = None
    for _ in range(max_steps):
        out = ctrl.update(x, y, yaw, dt, meas_v=last.v if last else 0.0,
                          meas_w=last.w if last else 0.0)
        last = out
        if out.done:
            return x, y, yaw, out
        # ideal integration of the commanded twist
        x += out.v * math.cos(yaw) * dt
        y += out.v * math.sin(yaw) * dt
        yaw = wrap_to_pi(yaw + out.w * dt)
    return x, y, yaw, last


# ---------- DISTANCE ----------
def test_distance_reaches_target_within_tol():
    ctrl = _make_controller()
    ctrl.begin(Goal(mode=Mode.DISTANCE, distance=1.0), 0.0, 0.0, 0.0)
    x, y, yaw, out = _simulate(ctrl, 0.0, 0.0, 0.0)
    assert out.done and out.success
    assert abs(x - 1.0) <= 0.02          # SPEC §7.1
    assert abs(out.error_remaining) <= 0.02


def test_distance_respects_v_max_and_accel():
    ctrl = _make_controller()
    ctrl.begin(Goal(mode=Mode.DISTANCE, distance=1.0), 0.0, 0.0, 0.0)
    dt = 0.02
    x = y = yaw = 0.0
    prev_v = 0.0
    last = None
    for _ in range(2000):
        out = ctrl.update(x, y, yaw, dt, meas_v=last.v if last else 0.0)
        last = out
        assert abs(out.v) <= 0.25 + 1e-9            # v_max
        assert abs(out.v - prev_v) <= 0.5 * dt + 1e-9  # a_max trapezoid
        prev_v = out.v
        if out.done:
            break
        x += out.v * math.cos(yaw) * dt
        y += out.v * math.sin(yaw) * dt
        yaw = wrap_to_pi(yaw + out.w * dt)


# ---------- ROTATE ----------
def test_rotate_ninety_degrees():
    ctrl = _make_controller()
    ctrl.begin(Goal(mode=Mode.ROTATE, yaw=math.pi / 2), 0.0, 0.0, 0.0)
    x, y, yaw, out = _simulate(ctrl, 0.0, 0.0, 0.0)
    assert out.done and out.success
    assert abs(wrap_to_pi(yaw - math.pi / 2)) <= 0.02   # SPEC §7.2


def test_rotate_takes_shortest_path_wrap():
    # target -170deg from +170deg start should wrap through 180, ~20deg move
    ctrl = _make_controller()
    start_yaw = math.radians(170.0)
    ctrl.begin(Goal(mode=Mode.ROTATE, yaw=math.radians(20.0)), 0.0, 0.0, start_yaw)
    x, y, yaw, out = _simulate(ctrl, 0.0, 0.0, start_yaw)
    assert out.done
    assert abs(wrap_to_pi(yaw - math.radians(190.0))) <= 0.02


# ---------- POSE ----------
def test_pose_relative_corner():
    ctrl = _make_controller()
    # target already resolved to odom by the node; here start at origin so
    # relative (0.5,0.5,0) == odom (0.5,0.5), final heading 0.
    goal = Goal(mode=Mode.POSE, target_x=0.5, target_y=0.5, yaw=0.0,
                has_final_heading=True)
    ctrl.begin(goal, 0.0, 0.0, 0.0)
    x, y, yaw, out = _simulate(ctrl, 0.0, 0.0, 0.0, max_steps=8000)
    assert out.done and out.success
    assert math.hypot(x - 0.5, y - 0.5) <= 0.03         # SPEC §7.3
    assert abs(wrap_to_pi(yaw - 0.0)) <= 0.03


def test_pose_position_only_no_final_turn():
    """has_final_heading=False (re-home): reach the point, never FINAL_TURN."""
    ctrl = _make_controller()
    goal = Goal(mode=Mode.POSE, target_x=0.5, target_y=0.5, yaw=0.0,
                has_final_heading=False)
    ctrl.begin(goal, 0.0, 0.0, 0.0)
    x = y = yaw = 0.0
    dt = 0.02
    last = None
    saw_final_turn = False
    for _ in range(8000):
        out = ctrl.update(x, y, yaw, dt,
                          meas_v=last.v if last else 0.0,
                          meas_w=last.w if last else 0.0)
        last = out
        if "final_turn" in out.stage:
            saw_final_turn = True
        if out.done:
            break
        x += out.v * math.cos(yaw) * dt
        y += out.v * math.sin(yaw) * dt
        yaw = wrap_to_pi(yaw + out.w * dt)
    assert out.done and out.success
    assert math.hypot(x - 0.5, y - 0.5) <= 0.03
    assert not saw_final_turn, "position-only POSE should skip FINAL_TURN"


def test_drive_drift_lead_sign_follows_turn_direction():
    """The pre-aim sign must auto-flip with the ALIGN turn direction (caster set
    by the turn), so one calibrated magnitude works for left- and right-turns."""
    ctrl = _make_controller()
    ctrl.tol.drive_drift_lead = 0.2
    # target to the LEFT of a robot facing +x → bearing +90°, ALIGN turns CCW
    ctrl.begin(Goal(mode=Mode.POSE, target_x=0.0, target_y=0.5, yaw=0.0,
                    has_final_heading=False), 0.0, 0.0, 0.0)
    ctrl.update(0.0, 0.0, 0.0, 0.02)
    assert ctrl._align_turn_dir == 1.0
    # target to the RIGHT → bearing -90°, ALIGN turns CW → sign flips
    ctrl.begin(Goal(mode=Mode.POSE, target_x=0.0, target_y=-0.5, yaw=0.0,
                    has_final_heading=False), 0.0, 0.0, 0.0)
    ctrl.update(0.0, 0.0, 0.0, 0.02)
    assert ctrl._align_turn_dir == -1.0


def test_pose_never_arcs_pure_v_or_omega():
    """Caster-safety invariant: POSE must never command v and omega together."""
    ctrl = _make_controller()
    goal = Goal(mode=Mode.POSE, target_x=0.5, target_y=0.5, yaw=0.0,
                has_final_heading=True)
    ctrl.begin(goal, 0.0, 0.0, 0.0)
    x = y = yaw = 0.0
    dt = 0.02
    last = None
    for _ in range(8000):
        out = ctrl.update(x, y, yaw, dt,
                          meas_v=last.v if last else 0.0,
                          meas_w=last.w if last else 0.0)
        last = out
        assert abs(out.v) < 1e-9 or abs(out.w) < 1e-9, \
            f"ARC at stage {out.stage}: v={out.v:.4f} w={out.w:.4f}"
        if out.done:
            break
        x += out.v * math.cos(yaw) * dt
        y += out.v * math.sin(yaw) * dt
        yaw = wrap_to_pi(yaw + out.w * dt)
    assert out.done and out.success


def test_pose_drive_goes_straight_no_realign_on_small_drift():
    """DRIVE must NOT re-align on small heading drift (that looped forever on the
    real chassis). It drives straight; only a ρ-rebound (mis-aim) re-approaches."""
    ctrl = _make_controller()
    goal = Goal(mode=Mode.POSE, target_x=1.0, target_y=0.0, yaw=0.0,
                has_final_heading=False)
    ctrl.begin(goal, 0.0, 0.0, 0.0)
    dt = 0.02
    x = y = yaw = 0.0
    last = None
    for _ in range(60):
        out = ctrl.update(x, y, yaw, dt,
                          meas_v=last.v if last else 0.0,
                          meas_w=last.w if last else 0.0)
        last = out
        x += out.v * math.cos(yaw) * dt
        y += out.v * math.sin(yaw) * dt
        yaw = wrap_to_pi(yaw + out.w * dt)
    assert out.stage == "pose_drive"
    # small heading drift (caster swing) — must keep driving, NOT re-align
    yaw = 0.1
    out = ctrl.update(x, y, yaw, dt, meas_v=last.v, meas_w=0.0)
    assert out.stage == "pose_drive", "re-aligned on small drift (would loop)"
    assert abs(out.w) < 1e-9, "DRIVE commanded omega (arc)"


# ---------- arrival latch ----------
def test_arrival_requires_consecutive_cycles():
    ctrl = _make_controller()
    ctrl.begin(Goal(mode=Mode.ROTATE, yaw=0.0), 0.0, 0.0, 0.0)
    # already at target & stopped: needs arrival_cycles (10) ticks to latch
    for i in range(9):
        out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_w=0.0)
        assert not out.done, f"latched too early at tick {i}"
    out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_w=0.0)
    assert out.done and out.success


def test_arrival_latch_resets_on_excursion():
    ctrl = _make_controller()
    ctrl.begin(Goal(mode=Mode.ROTATE, yaw=0.0), 0.0, 0.0, 0.0)
    for _ in range(5):
        ctrl.update(0.0, 0.0, 0.0, 0.02, meas_w=0.0)
    # excursion outside tol resets the counter
    out = ctrl.update(0.0, 0.0, 0.5, 0.02, meas_w=0.0)
    assert not out.done
    # back in band: must take a fresh 10 ticks
    for i in range(9):
        out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_w=0.0)
        assert not out.done, f"latched too early at tick {i}"
    out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_w=0.0)
    assert out.done


def test_not_arrived_while_still_moving():
    # in tolerance but still moving fast => not arrived (v_stop guard)
    ctrl = _make_controller()
    ctrl.begin(Goal(mode=Mode.DISTANCE, distance=0.0), 0.0, 0.0, 0.0)
    for _ in range(20):
        out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_v=0.5)  # moving
        assert not out.done


# ---------- deadzone floor ----------
def test_min_cmd_v_floor_lifts_small_commands():
    ctrl = _make_controller(limits={"min_cmd_v": 0.06})
    ctrl.begin(Goal(mode=Mode.DISTANCE, distance=0.05), 0.0, 0.0, 0.0)
    out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_v=0.0)
    # tiny error would give v ~ 0.06*... ; floor guarantees >= min_cmd_v
    # (first tick also rate-limited from 0, so check the floor is honoured
    #  once ramped — here just assert non-zero forward intent)
    assert out.v > 0.0


# ---------- dead-band kick (2026-06-10: floor AFTER slew) ----------
# 死区 (0, floor) 是非物理区间:指令在里面底盘纹丝不动,然后在某个不可预测的
# 点 stick-slip 暴冲。所以起步/反向必须一拍跳过死区直达 ±floor(separator
# binary_output 同思想),绝不在死区里慢爬。
def test_start_from_rest_kicks_to_floor_not_crawl():
    ctrl = _make_controller(limits={"min_cmd_v": 0.06})
    ctrl.begin(Goal(mode=Mode.DISTANCE, distance=1.0), 0.0, 0.0, 0.0)
    out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_v=0.0)
    assert out.v >= 0.06 - 1e-9, \
        f"first tick must kick to the floor, got crawl v={out.v:.3f}"


def test_rotate_start_kicks_to_w_floor():
    ctrl = _make_controller(limits={"min_cmd_w": 0.18})
    ctrl.begin(Goal(mode=Mode.ROTATE, yaw=math.pi / 2), 0.0, 0.0, 0.0)
    out = ctrl.update(0.0, 0.0, 0.0, 0.02, meas_w=0.0)
    assert abs(out.w) >= 0.18 - 1e-9, \
        f"first tick must kick to the w floor, got crawl w={out.w:.3f}"


def test_floor_reversal_crosses_dead_band_in_one_tick():
    """反向时不能卡在旧符号的 +floor 上(过零死锁),要一拍跳到 -floor。"""
    ctrl = _make_controller(limits={"min_cmd_v": 0.05})
    ctrl.v_prev = 0.05
    v, _ = ctrl._shape(-0.10, 0.0, 0.02, True, False)
    assert v <= -0.05 + 1e-9, f"stuck at old sign: v={v:.3f}"


def test_terminal_creep_holds_at_floor_while_active():
    """末端刹车帽压到地板以下时,仍要运动就保持 ≥ floor(不进死区爬行)。"""
    ctrl = _make_controller(limits={"min_cmd_v": 0.05})
    ctrl.v_prev = 0.05
    v, _ = ctrl._shape(0.02, 0.0, 0.02, True, False)
    assert v >= 0.05 - 1e-9


def test_inactive_ramps_to_zero_without_floor_lift():
    """到位去使能后正常斜坡降到 0,floor 不得把停车命令抬起来。"""
    ctrl = _make_controller(limits={"min_cmd_v": 0.05})
    ctrl.v_prev = 0.05
    v = 0.05
    for _ in range(10):
        v, _ = ctrl._shape(0.0, 0.0, 0.02, False, False)
    assert v == 0.0


def test_floor_above_max_rejected_at_construction():
    """floor > max 是矛盾配置(floor 在 clamp 后,会让输出恒超 max),
    必须在构造时报错(2026-06-10 实测最坏角死区 0.30 == 当时 w_max,差点踩上)。"""
    import pytest
    dist = PidGains(1.2, 0.0, 0.1, 0.3)
    yaw = PidGains(2.0, 0.0, 0.15, 0.5)
    with pytest.raises(ValueError):
        PositionController(Limits(w_max=0.30, min_cmd_w=0.32), dist, yaw,
                           Tolerances())
    with pytest.raises(ValueError):
        PositionController(Limits(v_max=0.25, min_cmd_v=0.30), dist, yaw,
                           Tolerances())
