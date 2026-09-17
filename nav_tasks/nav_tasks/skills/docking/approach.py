"""以静态地图校验预备点完整车身轮廓，与普通 POI 吸附策略分离."""

from typing import Sequence

from nav_tasks.domain.footprint import check_footprint, FootprintCheck, OccupancyGridView
from nav_tasks.skills.docking.ports import PlanarPose


def check_approach_footprint(
    grid: OccupancyGridView | None, global_frame: str,
    footprint: Sequence[tuple[float, float]], pose: PlanarPose, occupied_threshold: int,
) -> FootprintCheck:
    """先拒绝缺失或异帧地图，再校验放置后的完整车身多边形.

    帧名必须精确相等，本函数不查询 TF 或转换位姿。轮廓占用判定由
    check_footprint 执行；未知格沿用允许策略，不将其算作占用格。

    Args:
        grid: 最新静态地图的不可变快照，尚未收到时为 None。
        global_frame: 目标和预备点使用的显式全局坐标系。
        footprint: 导航基座坐标系中的车身多边形，不是中心点或净空半径。
        pose: 全局坐标系中预备点 x/y/yaw，单位米及弧度。
        occupied_threshold: 含等号的占用格阈值，未知格继续允许。

    Returns:
        原有准入结果、拒绝原因及存在时首个命中的占用格。
        缺图或异帧立即拒绝；其他合法性与越界检查交给轮廓检查器。
    """
    if grid is None:
        return FootprintCheck(False, 'static_map_not_received')
    if grid.frame_id != global_frame:
        return FootprintCheck(
            False, 'static_map_frame_mismatch:%s!=%s' % (grid.frame_id, global_frame))
    return check_footprint(grid, footprint, pose, occupied_threshold)
