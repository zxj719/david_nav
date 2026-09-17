"""保留旧轴对齐地图的同心圆环搜索，不依赖 ROS 订阅或完整路径规划."""

from dataclasses import dataclass, replace
import math
from typing import Callable

from nav_tasks.core.ports import PoseSource
from nav_tasks.domain.grid import GridMapView
from nav_tasks.domain.tasks import ResolvedGoal
from nav_tasks.domain.types import StoredPoi

Point = tuple[float, float]
GridSource = Callable[[], tuple[GridMapView | None, GridMapView | None]]


def _distance_xy(first: Point, second: Point) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


@dataclass(frozen=True)
class SnapSearch:
    """按旧代价地图格中心遍历顺序搜索同心圆环候选点.

    缺少任一地图时允许原锚点，非零格值不算空闲。周边净空是代价地图圆形
    采样规则，不是 Dock 完整轮廓检查。配置调整不等于完整数值合法性验证。

    Attributes:
        fallback_snap_radius: 后备搜索半径（米）。
        goal_clearance_radius: 候选点周边净空半径（米）。
        max_snap_radius: 扩圈半径上限（米）。
        snap_radius: 初始目标搜索半径（米）。
        snap_step: 扩圈半径增量（米）。
    """

    snap_radius: float = 0.5
    fallback_snap_radius: float = 1.0
    goal_clearance_radius: float = 0.1
    max_snap_radius: float = 3.0
    snap_step: float = 0.25

    def __post_init__(self) -> None:
        """把最大半径至少扩到后备半径，并把扩圈步长至少设为 0.05 米.

        不另外执行类型或有限性检查；调用方须提供适合有界搜索的配置。
        """
        object.__setattr__(self, 'max_snap_radius', max(
            self.max_snap_radius, self.fallback_snap_radius))
        object.__setattr__(self, 'snap_step', max(self.snap_step, 0.05))

    def resolve(
        self, anchor_xy: Point, robot_xy: Point | None,
        grid: GridMapView | None, costmap: GridMapView | None,
    ) -> Point | None:
        """返回原锚点或首个通过准入规则的扩圈格中心.

        先按半径从小到大搜索，每个圆环内才按机器人距离稳定排序；不是全局最近点
        或路径代价最优搜索。无机器人位姿时保留候选的行优先顺序。

        Args:
            anchor_xy: 请求的世界系锚点，单位米。
            robot_xy: 机器人世界系位置；None 表示不按机器人距离排序。
            grid: 可选静态占用栅格快照，采用旧轴对齐模型。
            costmap: 可选全局代价地图快照，采用同一坐标约定。

        Returns:
            准入的世界系坐标，搜索耗尽时为 None；缺任一地图时返回锚点。
            输入地图和配置应有效，数值运算错误不会转换成搜索耗尽。
        """
        if self.is_admissible(anchor_xy, grid, costmap):
            return anchor_xy
        radius = max(self.snap_radius, self.snap_step)
        while radius <= self.max_snap_radius + 1e-9:
            candidates = self._ring_candidates(anchor_xy, radius, costmap)
            if robot_xy is not None:
                candidates.sort(key=lambda xy: _distance_xy(xy, robot_xy))
            for candidate in candidates:
                if self.is_admissible(candidate, grid, costmap):
                    return candidate
            radius += self.snap_step
        return None

    def is_admissible(
        self, world_xy: Point, grid: GridMapView | None, costmap: GridMapView | None,
    ) -> bool:
        """执行旧零代价和圆形净空规则，不检查完整车身或路径.

        缺任一地图直接放行；否则要求两图中的目标格都恰为零。
        净空扫描只查代价地图；外接方形扫描中的任一格越图即拒绝，即使该格中心
        可能在圆外。此处保留既有边界规则，不把它表述成精确圆形几何包含。

        Args:
            world_xy: 世界系候选坐标，单位米。
            grid: 静态占用地图，或 None。
            costmap: 全局代价地图，或 None。

        Returns:
            在上述规则下是否准入；输入数值/地图错误仍可能抛异常。
        """
        if grid is None or costmap is None:
            return True
        grid_cell = grid.world_to_cell(*world_xy)
        global_cell = costmap.world_to_cell(*world_xy)
        if grid_cell is None or global_cell is None:
            return False
        if not grid.is_goal_free(*grid_cell) or not costmap.is_goal_free(*global_cell):
            return False
        if self.goal_clearance_radius <= 0.0:
            return True
        radius_cells = max(0, int(math.ceil(self.goal_clearance_radius / costmap.resolution)))
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x, y = global_cell[0] + dx, global_cell[1] + dy
                if not costmap.in_bounds(x, y):
                    return False
                candidate_xy = costmap.cell_center_world(x, y)
                if (_distance_xy(candidate_xy, world_xy) <= self.goal_clearance_radius
                        and not costmap.is_goal_free(x, y)):
                    return False
        return True

    @staticmethod
    def _ring_candidates(
        anchor_xy: Point, snap_radius: float, costmap: GridMapView | None,
    ) -> list[Point]:
        """按栅格行优先顺序生成圆环候选点，不在此检查占用或按机器人距离排序.

        Args:
            anchor_xy: 世界坐标锚点（米）。
            snap_radius: 搜索圆环半径（米）。
            costmap: 使用既有轴对齐约定的代价地图；None 时没有候选。

        Returns:
            圆环半径误差不超过半格加数值容差的栅格中心坐标；
            缺地图或锚点在图外时为空列表。
        """
        if costmap is None:
            return []
        anchor_cell = costmap.world_to_cell(*anchor_xy)
        if anchor_cell is None:
            return []
        radius_cells = max(1, int(math.ceil(snap_radius / costmap.resolution)))
        tolerance = 0.5 * costmap.resolution + 1e-9
        candidates = []
        for dy in range(-radius_cells - 1, radius_cells + 2):
            for dx in range(-radius_cells - 1, radius_cells + 2):
                x, y = anchor_cell[0] + dx, anchor_cell[1] + dy
                if not costmap.in_bounds(x, y):
                    continue
                world_xy = costmap.cell_center_world(x, y)
                if abs(_distance_xy(world_xy, anchor_xy) - snap_radius) <= tolerance:
                    candidates.append(world_xy)
        return candidates


class DirectGoalResolver:
    """在关闭吸附策略时原样返回点位，不作地图或路径可行性检查."""

    def resolve(self, poi: StoredPoi) -> ResolvedGoal:
        """原样返回请求锚点，不复制或修改其位姿.

        Args:
            poi: 请求点位。

        Returns:
            used_snap 为 False 且保留原 POI 的结果。
        """
        return ResolvedGoal(poi, False)

    def is_admissible(self, poi: StoredPoi) -> bool:
        """直接允许锚点，不读取任何占用信息.

        Args:
            poi: 请求点位，该策略不使用其字段。

        Returns:
            始终为 True，与旧直接解析策略一致。
        """
        return True


class GridGoalResolver:
    """把栅格搜索结果适配为 POI 值，不自行创建订阅或运动任务."""

    def __init__(
        self, grids: GridSource, poses: PoseSource, search: SnapSearch = SnapSearch(),
    ) -> None:
        """注入地图快照、位姿来源和搜索策略.

        Args:
            grids: 返回静态栅格和代价地图快照的提供者。
            poses: 用于给圆环候选排序的当前位姿来源。
            search: 纯搜索配置与准入策略。
        """
        self._grids = grids
        self._poses = poses
        self._search = search

    def resolve(self, poi: StoredPoi) -> ResolvedGoal | None:
        """寻找原锚点或吸附目标，只替换选中 POI 地图位姿的 x/y.

        地图快照获取错误会传播；仅机器人位姿读取的普通异常被忽略，退回行优先
        候选顺序。未作 TF 帧转换，调用方负责地图和 POI 坐标约定一致。

        Args:
            poi: 请求锚点，非位置字段保持不变。

        Returns:
            原始或吸附 POI，找不到合格候选时为 None；不证明存在可执行路径。

        Raises:
            Exception: 地图提供者、搜索或坐标计算失败。
        """
        anchor_xy = (poi.map_pose.x, poi.map_pose.y)
        grid, costmap = self._grids()
        if self._search.is_admissible(anchor_xy, grid, costmap):
            return ResolvedGoal(poi, False)
        try:
            pose = self._poses.current_map_pose()
            robot_xy = (pose.x, pose.y)
        except Exception:
            # Legacy policy: missing robot pose removes distance sorting only.
            robot_xy = None
        candidate = self._search.resolve(anchor_xy, robot_xy, grid, costmap)
        if candidate is None:
            return None
        snapped = replace(poi, map_pose=replace(poi.map_pose, x=candidate[0], y=candidate[1]))
        return ResolvedGoal(snapped, True)

    def is_admissible(self, poi: StoredPoi) -> bool:
        """用最新地图快照重新检查锚点，仍采用旧缺图放行策略.

        Args:
            poi: 要重新检查的原锚点。

        Returns:
            当前策略是否允许该锚点，不等于完整车身静态地图校验。

        Raises:
            Exception: 地图提供者或准入计算失败。
        """
        grid, costmap = self._grids()
        return self._search.is_admissible((poi.map_pose.x, poi.map_pose.y), grid, costmap)
