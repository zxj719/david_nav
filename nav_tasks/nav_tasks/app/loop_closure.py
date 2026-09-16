"""Supervised POI reprojection polling through localization and storage ports."""

import asyncio
from typing import Awaitable, Callable

from nav_tasks.core.ports import PoiRepository, PoseSource


async def watch_loop_closure(
    poses: PoseSource, repository: PoiRepository, poll_sec: float, *,
    translation_threshold_m: float = 0.05, yaw_threshold_rad: float = 0.03,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """Preserve exact-transform change detection and legacy reprojection thresholds.

    The composition root must supervise this coroutine. Missing TF is retried on
    the next poll; repository/session errors propagate. Only a successfully
    reprojected transform is remembered. Session-only changes with identical TF
    retain legacy skip behavior and are not silently fixed during migration.

    Args:
        poses: Read-only localization port.
        repository: POI persistence port.
        poll_sec: Existing poll interval, without adding new validation policy.
        translation_threshold_m: Inclusive translational no-change threshold.
        yaw_threshold_rad: Inclusive raw yaw no-change threshold.
        sleep: Injectable async wait; default uses the runtime event loop.

    Raises:
        asyncio.CancelledError: Supervisor shutdown cancels the loop.
        Exception: Repository or session identity lookup failed.
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
