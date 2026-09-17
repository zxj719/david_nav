"""Measure, judge and optionally correct yaw after phase two, without ROS dependencies.

先等待连续停稳；超时可报告最后位姿，但不能据此继续 Spin 补偿。
背景与线程所有权见 nav_tasks/docs/adr-0006-dock-thread.md。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from nav_tasks.skills.docking.settle import SettleSpec, wait_until_settled


@dataclass(frozen=True)
class DockTolerance:
    """Preserve the parameter names and meanings consumed by dock_navigator.

    Attributes:
        max_yaw_retries: 最多允许的偏航 Spin 补偿次数。
        xy_ok: 平面位置误差容差（米）。
        yaw_ok: 偏航误差容差（弧度）。
    """

    xy_ok: float = 0.10
    yaw_ok: float = 0.087           # rad, 约 5°
    max_yaw_retries: int = 2


@dataclass
class DockOutcome:
    """Finish verdict and measured settled pose, errors and correction count.

    Attributes:
        exy: 平面位置误差（米），缺失测量时沿用失败占位值。
        eyaw: 归一化有符号偏航误差（弧度）。
        n_spin: 已经尝试的 Spin 补偿次数。
        ok: 本次最终判定是否成功。
        pose: 判定时取得的 x/y/yaw 位姿；无法取得时为 None。
        reason: 冻结判词，例如 settled_ok、not_settled 或 no_pose。
        settled: 本次判定前的连续停稳检查是否通过。
    """

    ok: bool
    reason: str          # settled_ok / xy_out_of_tol / yaw_not_converged /
    # Remaining reason values: no_pose / not_settled.
    pose: tuple          # 判定所依据的位姿（停稳后的），取不到时为 None
    exy: float
    eyaw: float
    n_spin: int
    settled: bool


def _wrap(a: float) -> float:
    """把弧度差归一到半开区间 [-pi, pi)，正 pi 映射到负 pi.

    Args:
        a: 同一坐标系中计算的角度或角度差，单位弧度。

    Returns:
        按既有取模公式归一后的有符号角度。
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _errors(pose, goal):
    """计算实测位姿相对目标的二维距离及有符号航向误差.

    不应用成功阈值或修正运动；两位姿必须已在同一坐标系中。

    Args:
        pose: 实测 x/y/yaw，距离为米、角度为弧度。
        goal: 目标 x/y/yaw，使用相同单位和基准。

    Returns:
        非负平面距离与 pose.yaw 减 goal.yaw 归一到 [-pi, pi) 的弧度差。
    """
    exy = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    return exy, _wrap(pose[2] - goal[2])


def finish_docking(
    goal: tuple[float, float, float],
    pose_fn: Callable[[], tuple[float, float, float] | None],
    spin_fn: Callable[[float], bool],
    tol: DockTolerance,
    spec: SettleSpec,
    settle_timeout_sec: float,
    now_fn: Callable[[], float] | None = None,
    sleep_fn: Callable[[float], None] | None = None,
    on_event: Callable[..., None] | None = None,
) -> DockOutcome:
    """Finish docking only after settling, retaining the legacy correction policy.

    on_event(name, **kw) 可选，用来把过程打进 ROS 日志而不把 ROS 塞进本模块。

    Args:
        goal: Desired x/y/yaw pose.
        pose_fn: Current pose provider returning x/y/yaw or None.
        spin_fn: Injected spin operation, preserving the legacy ignored boolean result.
        tol: Position/yaw tolerances and correction budget.
        spec: Settling window and angular-rate thresholds.
        settle_timeout_sec: Maximum wait before each measurement.
        now_fn: Optional injected monotonic clock.
        sleep_fn: Optional injected sleeper.
        on_event: Optional callback receiving the unchanged event name and fields.

    Returns:
        The existing finish verdict, measurements and correction count.
    """
    def emit(name, **kw):
        if on_event is not None:
            on_event(name, **kw)

    settled, pose = wait_until_settled(
        pose_fn, spec, settle_timeout_sec, now_fn=now_fn, sleep_fn=sleep_fn)
    if pose is None:
        emit('no_pose')
        return DockOutcome(False, 'no_pose', None, float('nan'), float('nan'),
                           0, False)

    exy, eyaw = _errors(pose, goal)
    emit('measured', settled=settled, exy=exy, eyaw=eyaw)

    if not settled:
        # 没停稳就别补偿：nav2 的 Spin 对运动中的车会秒过，重试只会把误差推大。
        return DockOutcome(False, 'not_settled', pose, exy, eyaw, 0, False)

    if exy <= tol.xy_ok and abs(eyaw) <= tol.yaw_ok:
        return DockOutcome(True, 'settled_ok', pose, exy, eyaw, 0, True)

    if exy > tol.xy_ok:
        # 硬约定：入位位置没到就不再下发任何位移目标，只能调角度。
        return DockOutcome(False, 'xy_out_of_tol', pose, exy, eyaw, 0, True)

    n_spin = 0
    for _ in range(tol.max_yaw_retries):
        emit('spin', index=n_spin + 1, target_yaw=-eyaw)
        spin_fn(-eyaw)
        n_spin += 1

        settled, pose_after = wait_until_settled(
            pose_fn, spec, settle_timeout_sec, now_fn=now_fn, sleep_fn=sleep_fn)
        if pose_after is None:
            emit('no_pose')
            return DockOutcome(False, 'no_pose', pose, exy, eyaw, n_spin, False)
        pose = pose_after
        exy, eyaw = _errors(pose, goal)
        emit('after_spin', settled=settled, exy=exy, eyaw=eyaw)

        if not settled:
            return DockOutcome(False, 'not_settled', pose, exy, eyaw, n_spin, False)
        if abs(eyaw) <= tol.yaw_ok:
            return DockOutcome(True, 'settled_ok', pose, exy, eyaw, n_spin, True)

    return DockOutcome(False, 'yaw_not_converged', pose, exy, eyaw, n_spin, True)
