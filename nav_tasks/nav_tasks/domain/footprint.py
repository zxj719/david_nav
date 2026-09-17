"""Static-map footprint checks for the dock approach pose.

Docking must not send its first NavigateToPose goal when the complete chassis
footprint at that pose overlaps an occupied cell. The helpers here contain no
ROS imports so the safety decision can be tested without starting a ROS graph.
Unknown cells are not occupied cells; a footprint outside the map is rejected
because its clearance cannot be established.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
import math
from typing import Any, Sequence, Tuple

_EPS = 1e-9


@dataclass(frozen=True)
class OccupancyGridView:
    """Immutable fields needed from ``nav_msgs/OccupancyGrid``.

    Attributes:
        frame_id: Coordinate frame of the map.
        width: Grid width in cells.
        height: Grid height in cells.
        resolution: Cell size in metres.
        origin_x: X coordinate of the grid origin.
        origin_y: Y coordinate of the grid origin.
        origin_yaw: Rotation of the grid axes in its frame.
        data: Row-major occupancy values; -1 is unknown, 0 free, 100 occupied.
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
        """Copy the relevant fields from an OccupancyGrid-like message.

        Args:
            message: Object with the fields of ``nav_msgs/OccupancyGrid``.

        Returns:
            A view independent of later message mutation.
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
    """Result of checking one pose against a static occupancy grid.

    Attributes:
        clear: Whether the entire footprint is known to contain no occupied cell.
        reason: Stable short reason for logging and tests.
        occupied_cell: First colliding ``(x, y, value)`` cell, if any.
    """

    clear: bool
    reason: str
    occupied_cell: Tuple[int, int, int] | None = None


def parse_footprint(text: str) -> Tuple[Tuple[float, float], ...]:
    """Parse a ROS parameter string containing ``[[x, y], ...]``.

    Args:
        text: Python/YAML-compatible nested list string.

    Returns:
        Polygon points as immutable float pairs.

    Raises:
        ValueError: If the value is not a polygon with at least three points.
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
    """Check whether a posed footprint intersects any occupied static-map cell.

    Args:
        grid: Static occupancy grid.
        footprint: Polygon in the robot navigation-base frame.
        pose: Navigation-base ``(x, y, yaw)`` in ``grid.frame_id``.
        occupied_threshold: Non-negative occupancy values at or above this value
            are occupied. Unknown (-1) cells remain allowed.

    Returns:
        Clear, occupied, outside-map, or invalid-grid result.
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
    """Reject nonfinite, repeated, zero-area or self-intersecting body geometry."""
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
    """Rotate and translate a footprint into the world frame."""
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
    """Express one world point in the possibly rotated grid axes."""
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
    """Return whether two simple polygons overlap or touch."""
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
    """Return whether a point lies inside or on a simple polygon."""
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
    """Return whether two closed line segments intersect."""
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
