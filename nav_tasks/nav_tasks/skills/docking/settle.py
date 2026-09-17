"""Detect continuous settling from pose samples without ROS dependencies.

停稳判定必须覆盖完整窗口，不能把瞬时低角速度当作连续停稳。
桥接样本与测量依据见 nav_tasks/docs/adr-0006-dock-thread.md。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Sequence

# 浮点容差：窗口覆盖判定用，避免 0.4 vs 0.39999999 这种误判
_EPS = 1e-9


@dataclass(frozen=True)
class SettleSpec:
    """Thresholds for a continuously settled trailing sample window.

    Attributes:
        max_rate_rad_s: 相邻样本允许的最大表观偏航角速度（弧度/秒）。
        window_sec: 连续满足角速度限制所需覆盖的窗口（秒）。
        sample_period_sec: 请求位姿样本的轮询周期（秒）。
    """

    max_rate_rad_s: float = 0.05
    window_sec: float = 0.4
    sample_period_sec: float = 0.05


def _wrap(a: float) -> float:
    """把停稳窗口中的角度差归一到 [-pi, pi)，避免跨整周造成虚假跳变.

    Args:
        a: 已求得的角度差，单位弧度。

    Returns:
        既有取模公式得到的有符号角度差；正 pi 映射到负 pi。
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def is_settled(samples: Sequence[tuple[float, float]], spec: SettleSpec) -> bool:
    """Check continuous angular-rate stability across the complete trailing window.

    尾部样本必须覆盖窗口，窗口中相邻样本的表观角速度必须满足上限。
    角度差先 wrap，避免跨越 ±π 时将表示切换误作大角速度。
    窗口边界处理依据见 nav_tasks/docs/adr-0006-dock-thread.md。

    Args:
        samples: Time/yaw samples ordered by ascending time.
        spec: Continuous-window and angular-rate thresholds.

    Returns:
        Whether the trailing window is covered and every measured rate satisfies the threshold.
    """
    if len(samples) < 2:
        return False

    t_end = samples[-1][0]
    t_cut = t_end - spec.window_sec

    # 保留不晚于窗口起点的桥接样本，防止非整除采样把窗口截短。
    # 背景见 nav_tasks/docs/adr-0006-dock-thread.md。
    start = None
    for i, (t, _) in enumerate(samples):
        if t <= t_cut + _EPS:
            start = i
    if start is None:
        # 数据还不够老，窗口没被覆盖到 —— 这条正是"刚停 100ms 就宣布停稳"的闸门
        return False
    window = samples[start:]
    if len(window) < 2:
        return False

    for (t0, y0), (t1, y1) in zip(window, window[1:]):
        dt = t1 - t0
        if dt <= 0.0:
            continue
        if abs(_wrap(y1 - y0)) / dt > spec.max_rate_rad_s:
            return False
    return True


def wait_until_settled(
    pose_fn: Callable[[], tuple[float, float, float] | None],
    spec: SettleSpec,
    timeout_sec: float,
    now_fn: Callable[[], float] | None = None,
    sleep_fn: Callable[[float], None] | None = None,
) -> tuple[bool, tuple[float, float, float] | None]:
    """Poll until settled or timed out, retaining the last available pose.

    pose_fn : 无参可调用，返回 (x, y, yaw) 或 None（取不到位姿）。
    now_fn / sleep_fn : 注入时钟，方便确定性单测；默认用 time.monotonic / time.sleep。

    超时也把**最后一次拿到的位姿**交出去 —— 调用方需要它来报告实际误差，
    "等不到停稳"和"取不到位姿"是两回事。

    Args:
        pose_fn: Pose provider returning x/y/yaw or None.
        spec: Continuous-window and angular-rate thresholds.
        timeout_sec: Maximum polling duration.
        now_fn: Injected monotonic clock; defaults to time.monotonic.
        sleep_fn: Injected sleeper; defaults to time.sleep.

    Returns:
        Settled flag and the latest available pose, including when the wait times out.
    """
    now_fn = now_fn or time.monotonic
    sleep_fn = sleep_fn or time.sleep

    t_start = now_fn()
    samples = []
    last_pose = None
    while True:
        pose = pose_fn()
        if pose is not None:
            last_pose = pose
            samples.append((now_fn(), pose[2]))
            if is_settled(samples, spec):
                return True, last_pose
        if now_fn() - t_start >= timeout_sec:
            return False, last_pose
        sleep_fn(spec.sample_period_sec)
