"""在阶段二后等待采样停稳、测量误差并按既有策略补偿偏航.

等待超时可报告最后位姿，但不允许继续 Spin 补偿。
实体停稳证明与任务取消屏障不由本模块提供；线程所有权见
nav_tasks/docs/adr-0006-dock-thread.md。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from nav_tasks.skills.docking.settle import SettleSpec, wait_until_settled


@dataclass(frozen=True)
class DockTolerance:
    """保存既有位置、偏航容差及补偿次数，构造本身不校验.

    这些是纯辅助算法默认值，不可替代 David 精准模式的显式配置。

    Attributes:
        max_yaw_retries: 最多尝试的偏航 Spin 补偿次数，默认两次。
        xy_ok: 平面位置容差（米），默认 0.10。
        yaw_ok: 偏航容差（弧度），默认 0.087（约 5 度）。
    """

    xy_ok: float = 0.10
    yaw_ok: float = 0.087           # rad, 约 5°
    max_yaw_retries: int = 2


@dataclass
class DockOutcome:
    """保存最终算法判词、最后可用测量与补偿计数的可变记录.

    失败时位姿可能尚未停稳；后续取不到位姿时可保留上次测量。
    本记录不证明动作清理完成或实体车静止。

    Attributes:
        exy: 所保留位姿的平面误差（米），从未测得时为 NaN。
        eyaw: 所保留位姿的归一化有符号偏航误差（弧度），缺测时为 NaN。
        n_spin: 已尝试的 Spin 补偿次数。
        ok: 既有精调算法是否判定成功。
        pose: 最后可用的 x/y/yaw；从未取到时为 None，不保证来自本次等待。
        reason: 冻结判词，例如 settled_ok、not_settled 或 no_pose。
        settled: 最近一次等待的采样停稳检查是否通过。
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
    """等待采样停稳后判断误差并执行有限偏航补偿，保留旧策略.

    首次等待失败时不补偿；初次位置超差也不再下发位移目标。
    Spin 的布尔返回值沿用忽略策略，之后重新等待并测量；既有循环仅再检查
    偏航容差，不重复以位置容差拒绝补偿后的结果。显式取消和清理屏障由外层负责。
    位姿、旋转、时钟、休眠及事件回调异常原样传播。

    Args:
        goal: 与实测位姿同一坐标系的目标 x/y/yaw，单位米及弧度。
        pose_fn: 当前位姿读取器，返回 x/y/yaw 或 None。
        spin_fn: 注入的相对旋转操作，返回的布尔结果按旧策略不参与判词。
        tol: 位置、偏航容差与补偿次数预算。
        spec: 采样停稳窗口和表观角速度限制。
        settle_timeout_sec: 每次测量前的轮询预算（秒），不是硬墙钟上限。
        now_fn: 可选的注入单调时钟。
        sleep_fn: 可选的注入休眠器。
        on_event: 可选回调，以原有事件名及具名字段接收过程事件。

    Returns:
        既有判词、测量与补偿计数；超时或后续缺测可保留上次位姿。
        成功是算法判定，不证明完整任务取消屏障或实体停稳。
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
