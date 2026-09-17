"""经定位与存储端口轮询回环重投影，由组装根托管协程."""

import asyncio
from typing import Awaitable, Callable

from nav_tasks.core.ports import PoiRepository, PoseSource


async def watch_loop_closure(
    poses: PoseSource, repository: PoiRepository, poll_sec: float, *,
    translation_threshold_m: float = 0.05, yaw_threshold_rad: float = 0.03,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """按变换精确变化触发 POI 重投影，保留旧阈值与会话跳过策略.

    每轮先等待；读取变换的普通异常被忽略，下一轮重试。变换与上次精确相等
    时直接跳过，不再查询会话；仅会话变化但变换相同不会触发重投影。
    仓库调用正常返回后即记住该变换，即使没有记录超过阈值。
    本协程不检查位姿新鲜度或校验轮询间隔，不查询或改动定位栈配置。

    Args:
        poses: 只读定位端口，提供地图相对里程计变换及会话标识。
        repository: 同步 POI 持久化端口，执行当前会话重投影。
        poll_sec: 沿用的轮询间隔（秒），不在此增加校验。
        translation_threshold_m: 平面变化不超过此值时可视为未变化（米）。
        yaw_threshold_rad: 未归一化 yaw 差不超过此值时可视为未变化（弧度）。
        sleep: 可注入异步等待函数，默认使用运行循环的 asyncio.sleep。

    Raises:
        asyncio.CancelledError: 协程或注入等待被取消。
        Exception: 等待、会话查询或仓库调用失败，交由外层 supervisor 处理。
    """
    last_transform = None
    while True:
        await sleep(poll_sec)
        try:
            transform = poses.current_map_from_odom()
        except Exception:
            continue
        if transform == last_transform:
            continue
        repository.reproject_session(
            poses.current_slam_session_id(), transform,
            translation_threshold_m, yaw_threshold_rad)
        last_transform = transform
