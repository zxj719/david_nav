"""根据位姿采样的偏航变化判定完整尾部窗口，不依赖 ROS.

保留窗口起点的桥接样本，避免非整除采样缩短窗口。
这里只判断观测偏航变化，不验证实体零速度、平移或定位数据新鲜度。
历史测量与窗口依据见 nav_tasks/docs/adr-0006-dock-thread.md。
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
    """保存偏航采样停稳阈值与轮询参数，构造本身不校验数值.

    默认角速度上限为 0.05 rad/s，窗口为 0.4 秒，采样周期为 0.05 秒。
    调用方负责合法配置；该不可变记录不能证明样本质量或实体停稳。

    Attributes:
        max_rate_rad_s: 相邻正时间间隔样本允许的最大表观偏航角速度（弧度/秒）。
        window_sec: 尾部样本需要覆盖的窗口（秒）。
        sample_period_sec: 请求位姿样本的轮询间隔（秒），不是实际采样周期保证。
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
    """检查尾部窗口覆盖及相邻有效时间间隔的表观偏航角速度.

    取最后一个不晚于窗口起点加 1e-9 秒容差的桥接样本，角度差先归一。
    少于两个窗口样本或缺少桥接样本返回 False；非正时间间隔沿用跳过策略。
    不校验排序、有限数值、最大采样间隙或位姿新鲜度，也不判断平移速度。
    桥接依据见 nav_tasks/docs/adr-0006-dock-thread.md。

    Args:
        samples: 调用方保证按时间递增的时间/偏航样本，单位秒及弧度。
        spec: 调用方提供的合法窗口和角速度限制。

    Returns:
        窗口满足覆盖条件且所有正时间间隔的观测角速度均未超上限时为 True。
        这是采样判定，不是连续物理零速度证明。
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
    """同步轮询偏航停稳或超时，并保留最后可用位姿用于误差报告.

    位姿缺失时不追加样本，也不清除之前的样本；停稳检查先于超时检查。
    超时不是包含回调执行的硬墙钟上限，阻塞回调会延长等待。
    本函数不内置取消或资源清理；受管工作线程通过注入回调协作停止。
    回调或无效输入导致的异常原样传播，不把异常当作无位姿。

    Args:
        pose_fn: 无参位姿读取器，返回同一坐标系的 x/y/yaw（米、弧度）或 None。
        spec: 偏航窗口、角速度限制和请求采样间隔。
        timeout_sec: 每次轮询后检查的等待预算（秒）。
        now_fn: 可注入单调时钟，默认 time.monotonic。
        sleep_fn: 可注入休眠器，默认 time.sleep。

    Returns:
        采样停稳标志与最后可用位姿；超时仍保留该位姿，从未取到则为 None。
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
