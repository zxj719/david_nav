"""提供 Dock 预备点的静态地图完整车身轮廓校验.

调用方应在首次下发 NavigateToPose 前检查预备位姿。此模块不导入 ROS，
不发送目标、不查询 TF；使用带原点旋转的栅格模型，把车身多边形与
占用格闭矩形相交作为拒绝条件。未知格沿用允许策略，按边界容差判定的
越图轮廓拒绝；clear 不等于全部轮廓都落在已知空闲格。
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
import math
from typing import Any, Sequence, Tuple

_EPS = 1e-9


@dataclass(frozen=True)
class OccupancyGridView:
    """保存 Dock 校验使用的不可变栅格字段快照，支持原点旋转.

    构造本身不保证消息合法；check_footprint 另行检查基本几何和数据长度，
    但不会检查所有格值的取值范围。

    Attributes:
        frame_id: 地图坐标系名称。
        width: 栅格列数。
        height: 栅格行数。
        resolution: 每格边长，单位米。
        origin_x: 栅格原点在地图系中的 x，单位米。
        origin_y: 栅格原点在地图系中的 y，单位米。
        origin_yaw: 栅格轴相对地图系的偏航角，单位弧度。
        data: 行优先格值元组；约定 -1 未知、0 空闲、100 占用。
    """

    frame_id: str
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Tuple[int, ...]

    @classmethod
    def from_message(cls, message: Any) -> 'OccupancyGridView':
        """从 OccupancyGrid 兼容对象复制并转换相关字段.

        原点偏航角沿用直接 atan2 算式，不归一化四元数；这里是字段适配而非
        完整合法性检查。字段访问或类型转换异常向调用方传播。

        Args:
            message: 具有 nav_msgs/OccupancyGrid 相应字段的对象。

        Returns:
            与后续消息修改独立的快照；data 已复制为整数元组。
        """
        origin = message.info.origin
        orientation = origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return cls(
            frame_id=str(message.header.frame_id),
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(origin.position.x),
            origin_y=float(origin.position.y),
            origin_yaw=yaw,
            data=tuple(int(value) for value in message.data),
        )


@dataclass(frozen=True)
class FootprintCheck:
    """表示一个位姿的轮廓静态地图检查结果，不证明实机可达或停稳.

    Attributes:
        clear: 在当前阈值和边界策略下通过检查；未知格允许，不表示全部已知空闲。
        reason: 供日志和测试使用的稳定短原因。
        occupied_cell: 首个相交占用格的 (列, 行, 格值)，无此格时为 None。
    """

    clear: bool
    reason: str
    occupied_cell: Tuple[int, int, int] | None = None


def parse_footprint(text: str) -> Tuple[Tuple[float, float], ...]:
    """解析表示车身简单多边形的 Python 字面量参数字符串.

    使用 ast.literal_eval，不是通用 YAML 解析器。拒绝布尔坐标、非有限数值、
    重复顶点、退化面积和非相邻边相交；首点不应在末尾再次写出。

    Args:
        text: 类似 [[x, y], ...] 的列表或元组字面量字符串。

    Returns:
        至少三个顶点组成的不可变浮点坐标对元组，坐标单位由调用方约定。

    Raises:
        ValueError: 字面量语法或受检多边形条件不满足。
        OverflowError: 过大整数字面量无法完成有限性检查或浮点转换。
    """
    try:
        value = ast.literal_eval(text)
    except (SyntaxError, ValueError) as error:
        raise ValueError(f'invalid footprint: {error}') from error
    valid = (
        isinstance(value, (list, tuple))
        and len(value) >= 3
        and all(
            isinstance(point, (list, tuple))
            and len(point) == 2
            and all(
                isinstance(coordinate, (int, float)) and not isinstance(coordinate, bool)
                and math.isfinite(coordinate)
                for coordinate in point)
            for point in value)
    )
    if not valid or not _valid_polygon(value):
        raise ValueError('footprint must contain at least three numeric [x, y] points')
    return tuple((float(point[0]), float(point[1])) for point in value)


def check_footprint(
    grid: OccupancyGridView,
    footprint: Sequence[Tuple[float, float]],
    pose: Tuple[float, float, float],
    occupied_threshold: int = 50,
) -> FootprintCheck:
    """检查指定车身位姿的完整轮廓是否与静态地图占用格相交.

    支持地图原点旋转；调用方必须保证 pose 属于 grid.frame_id，本函数不做
    TF 查询或帧匹配。接触占用格边界也算相交；未知格 -1 保持允许。
    按既有 _EPS 容差判定越图，正向外边界被拒绝。格值范围、严格输入类型及
    中间运算溢出不在完整验证范围，不能将本函数当作任意对象的安全解析器。

    Args:
        grid: 静态占用栅格快照，不能为 None；缺图由调用方处理。
        footprint: 导航旋转中心帧中的车身二维多边形，单位米。
        pose: 导航旋转中心在 grid.frame_id 中的 (x, y, yaw)，单位米/弧度。
        occupied_threshold: 0..100 内的阈值，达到或超过即占用；默认 50。

    Returns:
        clear 或 invalid_static_map、invalid_occupied_threshold、invalid_pose、
        invalid_footprint、footprint_outside_static_map、occupied_static_cell 原因。
        相交占用格按先行后列顺序返回第一个，不代表唯一碰撞格。
    """
    if (
        grid.width <= 0
        or grid.height <= 0
        or grid.resolution <= 0.0
        or not all(math.isfinite(value) for value in (
            grid.resolution, grid.origin_x, grid.origin_y, grid.origin_yaw))
        or len(grid.data) != grid.width * grid.height
    ):
        return FootprintCheck(False, 'invalid_static_map')
    if not 0 <= occupied_threshold <= 100:
        return FootprintCheck(False, 'invalid_occupied_threshold')
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        return FootprintCheck(False, 'invalid_pose')
    if not _valid_polygon(footprint):
        return FootprintCheck(False, 'invalid_footprint')

    polygon = [_world_to_grid_local(grid, point) for point in _pose_polygon(footprint, pose)]
    map_width = grid.width * grid.resolution
    map_height = grid.height * grid.resolution
    if any(
        x < -_EPS or y < -_EPS or x >= map_width - _EPS or y >= map_height - _EPS
        for x, y in polygon
    ):
        return FootprintCheck(False, 'footprint_outside_static_map')

    # Closed cells on both sides of an exact grid line touch the footprint.
    min_x = max(0, int(math.floor((min(point[0] for point in polygon) - _EPS) / grid.resolution)))
    min_y = max(0, int(math.floor((min(point[1] for point in polygon) - _EPS) / grid.resolution)))
    max_x = min(
        grid.width - 1,
        int(math.floor((max(point[0] for point in polygon) + _EPS) / grid.resolution)),
    )
    max_y = min(
        grid.height - 1,
        int(math.floor((max(point[1] for point in polygon) + _EPS) / grid.resolution)),
    )
    for cell_y in range(min_y, max_y + 1):
        for cell_x in range(min_x, max_x + 1):
            value = grid.data[cell_y * grid.width + cell_x]
            if value < occupied_threshold:
                continue
            cell = _cell_polygon(cell_x, cell_y, grid.resolution)
            if _polygons_intersect(polygon, cell):
                return FootprintCheck(
                    False, 'occupied_static_cell', (cell_x, cell_y, value))
    return FootprintCheck(True, 'clear')


def _valid_polygon(points: Sequence[Tuple[float, float]]) -> bool:
    """检查车身多边形是否有限、无重复顶点、有面积且没有非邻边相交.

    Args:
        points: 按边界顺序排列的二维数值点序列；首点不在末尾重复。
            调用方负责提供可取长度且坐标支持有限性检查的对象。

    Returns:
        至少三个点、各坐标有限、顶点互异、两倍有符号面积绝对值大于 _EPS，
        且所有非邻边均不相交时为 True。不是任意输入类型的解析器。
    """
    if len(points) < 3 or any(
        len(point) != 2 or not all(math.isfinite(value) for value in point)
        for point in points
    ):
        return False
    vertices = tuple(tuple(point) for point in points)
    if len(set(vertices)) != len(vertices):
        return False
    edges = tuple(zip(vertices, vertices[1:] + vertices[:1]))
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in edges)
    if abs(area) <= _EPS:
        return False
    for index, (start, end) in enumerate(edges):
        for other_index in range(index + 2, len(edges)):
            if index == 0 and other_index == len(edges) - 1:
                continue
            if _segments_intersect(start, end, *edges[other_index]):
                return False
    return True


def _pose_polygon(
    footprint: Sequence[Tuple[float, float]], pose: Tuple[float, float, float]
) -> Tuple[Tuple[float, float], ...]:
    """将导航基准局部轮廓按平面位姿旋转并平移到地图参考帧.

    Args:
        footprint: 轮廓在机器人导航基准帧中的二维顶点，单位米。
        pose: 导航基准在目标参考帧中的 x、y、yaw，单位米、米、弧度；
            调用方应保证输入合法。

    Returns:
        与输入顶点顺序相同的不可变二维点元组，不闭合追加首点。
    """
    x, y, yaw = pose
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tuple(
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in footprint
    )


def _world_to_grid_local(
    grid: OccupancyGridView, point: Tuple[float, float]
) -> Tuple[float, float]:
    """把地图参考帧中的点换算到可能旋转的栅格原点坐标系.

    先减原点平移，再施加原点 yaw 的逆旋转；不除以分辨率，
    因此返回的不是格子下标。

    Args:
        grid: 含原点平移和弧度制 yaw 的静态地图视图。
        point: 已在 grid.frame_id 中表达的二维点，单位米。

    Returns:
        相对于地图原点和局部栅格轴的二维米制坐标。
    """
    dx = point[0] - grid.origin_x
    dy = point[1] - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def _cell_polygon(x: int, y: int, resolution: float) -> Tuple[Tuple[float, float], ...]:
    """把栅格下标转换为地图局部米制坐标中的四角矩形.

    不应用地图世界原点或原点旋转；调用方应先把待比较轮廓转换到同一局部系。
    分辨率应已检查为正，首点不在末尾重复。

    Args:
        x: 栅格列下标。
        y: 栅格行下标。
        resolution: 每个栅格的边长，单位米。

    Returns:
        从左下角开始逆时针排列的四个局部二维顶点。
    """
    low_x = x * resolution
    low_y = y * resolution
    high_x = low_x + resolution
    high_y = low_y + resolution
    return ((low_x, low_y), (high_x, low_y), (high_x, high_y), (low_x, high_y))


def _polygons_intersect(
    first: Sequence[Tuple[float, float]], second: Sequence[Tuple[float, float]]
) -> bool:
    """用顶点包含和边交叉判断两个简单多边形是否重叠或接触.

    包含边界接触；不在此验证多边形是否简单、有限或非空。

    Args:
        first: 同一平面坐标系中的第一组有序顶点，支持切片和拼接。
        second: 第二组有序顶点，与第一组采用相同单位且不重复末尾首点。

    Returns:
        任一顶点落在另一多边形内或边上，或任意两边相交时为 True。
    """
    if any(_point_in_polygon(point, second) for point in first):
        return True
    if any(_point_in_polygon(point, first) for point in second):
        return True
    first_edges = zip(first, first[1:] + first[:1])
    second_edges = tuple(zip(second, second[1:] + second[:1]))
    return any(
        _segments_intersect(start, end, other_start, other_end)
        for start, end in first_edges
        for other_start, other_end in second_edges
    )


def _point_in_polygon(
    point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]
) -> bool:
    """以水平射线奇偶法判断点是否位于简单多边形内部或边上.

    边界先按 _point_on_segment 的容差检查；其余情况只在跨越射线时
    翻转内部标志。调用方保证多边形有效。

    Args:
        point: 待检查的二维点。
        polygon: 同坐标系、同单位的有序边界顶点，首点不在末尾重复。

    Returns:
        点在内部或带既有容差的边界上时为 True，否则为 False。
    """
    x, y = point
    inside = False
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        if _point_on_segment(point, previous, current):
            return True
        if (current[1] > y) != (previous[1] > y):
            crossing_x = (
                (previous[0] - current[0]) * (y - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if x < crossing_x:
                inside = not inside
    return inside


def _point_on_segment(
    point: Tuple[float, float],
    start: Tuple[float, float],
    end: Tuple[float, float],
) -> bool:
    """按既有叉积与点积容差检查点是否落在闭线段上.

    Args:
        point: 待检查的平面点，与端点采用相同坐标单位。
        start: 线段起点。
        end: 线段终点；该计算假定边不退化为零长度。

    Returns:
        叉积绝对值不超过 _EPS，且点积在带 _EPS 容差的线段范围内时为 True。
    """
    cross = (
        (point[0] - start[0]) * (end[1] - start[1])
        - (point[1] - start[1]) * (end[0] - start[0])
    )
    if abs(cross) > _EPS:
        return False
    dot = (
        (point[0] - start[0]) * (end[0] - start[0])
        + (point[1] - start[1]) * (end[1] - start[1])
    )
    length_squared = (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
    return -_EPS <= dot <= length_squared + _EPS


def _segments_intersect(
    first_start: Tuple[float, float],
    first_end: Tuple[float, float],
    second_start: Tuple[float, float],
    second_end: Tuple[float, float],
) -> bool:
    """用方向叉积与边界容差判断两条闭线段是否相交.

    严格跨越直接算相交；共线或端点接触再按 _EPS 与点在线段上的检查处理。
    输入应为有限、非退化边，本函数不先验证几何合法性。

    Args:
        first_start: 第一条线段的二维起点。
        first_end: 第一条线段的二维终点。
        second_start: 第二条线段的二维起点，与第一条线段处于同坐标系。
        second_end: 第二条线段的二维终点，所有点采用同一长度单位。

    Returns:
        两条闭线段交叉、重叠或在既有容差内接触时为 True。
    """
    def orientation(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    values = (
        orientation(first_start, first_end, second_start),
        orientation(first_start, first_end, second_end),
        orientation(second_start, second_end, first_start),
        orientation(second_start, second_end, first_end),
    )
    if values[0] * values[1] < 0.0 and values[2] * values[3] < 0.0:
        return True
    return (
        abs(values[0]) <= _EPS and _point_on_segment(second_start, first_start, first_end)
        or abs(values[1]) <= _EPS and _point_on_segment(second_end, first_start, first_end)
        or abs(values[2]) <= _EPS and _point_on_segment(first_start, second_start, second_end)
        or abs(values[3]) <= _EPS and _point_on_segment(first_end, second_start, second_end)
    )
