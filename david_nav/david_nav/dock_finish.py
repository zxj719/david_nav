"""dock_finish —— 阶段2 之后的"测量 -> 判定 -> 纯旋转补偿"。纯逻辑，无 ROS，可单测。

## 这段逻辑为什么要单独拎出来

2026-09-10 dock_bench 首次冒烟（5 轮，3 轮 YAW_NOT_CONVERGED）实测：原来的实现把
这段逻辑内联在 `dock_navigator.execute_docking` 里，一行 ROS 都绕不开，导致
**唯一的验证手段是开着车跑**。而它的 bug 恰恰是时序性的（在运动中测量），
跑一次要 40s、还看天吃饭。抽出来 + 注入 pose/spin 之后，用一个带惯性的假底盘
就能在毫秒级把这类时序 bug 钉死。

## 修的是什么

原实现在阶段2 的 NavigateToPose 一报 SUCCEEDED 就立刻取 TF 测 eyaw，而此时车仍以
**0.55 rad/s** 在转（末端 RotateToGoal 满速收朝向的尾巴）：

    25.66 阶段2 SUCCEEDED, 入位实测 eyaw=6.8°   ← 运动中的瞬时值，不是终点
    25.67 Spin(-6.8°)
    25.77 Spin#1: SUCCEEDED                     ← 100ms 就"成功"
    25.77 旋转后 eyaw=13.9°                      ← 惯性又转过去 7°
    25.88 Spin#2: ABORTED → 失败 eyaw=19.6°

220ms 内两次补偿把误差从 6.8° 推到 19.6°。**nav2 的 `Spin` 用 `abs(relative_yaw)`
判完成**，对一台还在反向转的车，累计转角绝对值瞬间超过目标量 → 立刻 SUCCEEDED。
所以"车没停稳时 Spin 在原理上就不可能起作用，重试只会更糟"。

修法：**每次测量之前、每次 Spin 之后，都先等车真正停稳**（见 dock_settle）。
没停稳就不下发 Spin —— 宁可如实报 `not_settled`，也不做一次注定使误差变大的补偿。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from david_nav.dock_settle import SettleSpec, wait_until_settled


@dataclass(frozen=True)
class DockTolerance:
    """与 dock_navigator 的参数同名同义，避免两处漂移。"""

    xy_ok: float = 0.10
    yaw_ok: float = 0.087           # rad, 约 5°
    max_yaw_retries: int = 2


@dataclass
class DockOutcome:
    ok: bool
    reason: str          # settled_ok / xy_out_of_tol / yaw_not_converged /
                         # no_pose / not_settled
    pose: tuple          # 判定所依据的位姿（停稳后的），取不到时为 None
    exy: float
    eyaw: float
    n_spin: int
    settled: bool


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _errors(pose, goal):
    exy = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    return exy, _wrap(pose[2] - goal[2])


def finish_docking(goal, pose_fn, spin_fn, tol: DockTolerance, spec: SettleSpec,
                   settle_timeout_sec: float, now_fn=None, sleep_fn=None,
                   on_event=None) -> DockOutcome:
    """goal = (x, y, yaw)；pose_fn() -> (x,y,yaw)|None；spin_fn(target_yaw) -> bool。

    on_event(name, **kw) 可选，用来把过程打进 ROS 日志而不把 ROS 塞进本模块。
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
