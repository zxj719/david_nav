"""仅通过公共 TaskApi 观察任务，保留既有状态文本和输出节奏."""

import asyncio
import time
from typing import Awaitable, Callable, Protocol

from nav_tasks.api import PoseRecord, PoseUnavailableError, TaskApi, TaskSnapshot


class StatusPoseSource(Protocol):
    """定义状态显示所需的最小位姿观察接口，不依赖 ROS 实现."""

    def current_map_pose(self) -> PoseRecord:
        """读取地图帧下最新可用位姿，不执行导航或保证任务成功.

        Returns:
            地图帧中的位姿记录，位置为米、航向为弧度。

        Raises:
            PoseUnavailableError: 位姿源无法提供可用样本，具体实现负责可用性检查。
        """
        ...


def format_status_line(snapshot: TaskSnapshot, poses: StatusPoseSource) -> str:
    """从公共任务快照生成冻结 STATUS 文本，不读取任务管理器内部状态.

    只把位姿读取及格式化的普通异常转为 pose=n/a 标记；
    快照状态或距离格式本身的错误不在该隔离范围内。

    Args:
        snapshot: 公共任务状态快照。
        poses: 当前位姿源，位姿不可用时显示错误类型而非虚构坐标。

    Returns:
        不含日志前缀和换行的既有状态行；位姿 x/y 为米，yaw 为弧度。
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
        if isinstance(error, PoseUnavailableError) and error.__cause__ is not None:
            error = error.__cause__
        pose_text = f'pose=n/a ({type(error).__name__})'
    return (
        f'STATUS state={snapshot.state.value} poi={poi} phase={phase} '
        f'd2g={distance} {pose_text}')


async def print_status_loop(
    tasks: TaskApi, poses: StatusPoseSource, interval_sec: float,
    emit: Callable[[str], None], *, now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """保留四分之一秒轮询、活动任务心跳和空闲状态变化时输出的策略.

    由运行器持有本协程，此函数不创建后台任务。
    比较键为状态、目标 ID、阶段，不含请求 ID 或位姿；首次快照会输出。
    有任务时还按间隔触发，不在此新增间隔验证或实际 CPU 占用保证。

    Args:
        tasks: 公共任务观察 API。
        poses: 当前位姿源。
        interval_sec: 存在任务 ID 时的状态心跳间隔，单位秒。
        emit: 接收完整无前缀文本的日志函数。
        now: 可注入的单调秒数读取函数。
        sleep: 可注入的秒制异步等待，默认 asyncio.sleep。

    Raises:
        asyncio.CancelledError: 运行器关闭时取消轮询。
        Exception: 状态读取、等待、计时或日志失败；位姿错误按格式化函数规则降级。
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
