"""Legacy status text and cadence observed exclusively through public TaskApi."""

import asyncio
import time
from typing import Awaitable, Callable, Protocol

from nav_tasks.api import PoseRecord, TaskApi, TaskSnapshot


class StatusPoseSource(Protocol):
    """Minimal pose observation contract, independent of ROS implementation."""

    def current_map_pose(self) -> PoseRecord:
        """Read the latest pose, raising when it is unavailable.

        Returns:
            Current map-frame pose.
        """
        ...


def format_status_line(snapshot: TaskSnapshot, poses: StatusPoseSource) -> str:
    """Render frozen STATUS wording without reading task-manager internals.

    Args:
        snapshot: Immutable public task observation.
        poses: Current pose source; pose-read errors become a readable marker.

    Returns:
        Legacy status line without logger prefix or newline.
    """
    if snapshot.task_id is not None:
        poi = snapshot.target_id if snapshot.target_id is not None else '-'
        phase = snapshot.phase if snapshot.phase is not None else '-'
        distance = f'{snapshot.remaining_distance:.2f}m'
    else:
        poi, phase, distance = '-', '-', '-'
    try:
        pose = poses.current_map_pose()
        pose_text = f'x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}rad'
    except Exception as error:
        pose_text = f'pose=n/a ({type(error).__name__})'
    return (
        f'STATUS state={snapshot.state.value} poi={poi} phase={phase} '
        f'd2g={distance} {pose_text}')


async def print_status_loop(
    tasks: TaskApi, poses: StatusPoseSource, interval_sec: float,
    emit: Callable[[str], None], *, now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """Keep legacy quarter-second polling, active heartbeat and transition-only idle.

    The runtime supervisor must own this coroutine; it is never started here.
    No new interval validation is added during compatibility migration.

    Args:
        tasks: Public task observation API.
        poses: Current pose source.
        interval_sec: Heartbeat interval while any task owns motion.
        emit: Unprefixed complete-line sink, normally the runtime logger.
        now: Injectable monotonic time reader.
        sleep: Injectable asynchronous wait, defaulting to asyncio.sleep.

    Raises:
        asyncio.CancelledError: Runtime shutdown cancels the polling loop.
    """
    last_key = None
    last_print_time = 0.0
    while True:
        await sleep(0.25)
        snapshot = tasks.status()
        key = (snapshot.state.value, snapshot.target_id, snapshot.phase)
        current = now()
        due = snapshot.task_id is not None and current - last_print_time >= interval_sec
        if key != last_key or due:
            emit(format_status_line(snapshot, poses))
            last_key = key
            last_print_time = current
