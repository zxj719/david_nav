"""保护已审坐标/栅格摘要及既有边界；不替代语义审查或实机验收."""

import ast
import math
from pathlib import Path
import re
from types import SimpleNamespace

import pytest

from nav_tasks.domain.footprint import (
    check_footprint, OccupancyGridView, parse_footprint,
)
from nav_tasks.domain.geometry import legacy_yaw_from_quaternion, transform_pose
from nav_tasks.domain.grid import GridMapView
from nav_tasks.domain.types import PoseRecord, Transform2D


@pytest.mark.parametrize('path, qualified_name', [
    ('geometry.py', '<module>'),
    ('geometry.py', 'wrap_to_pi'),
    ('geometry.py', 'yaw_from_quaternion'),
    ('geometry.py', 'quaternion_from_yaw'),
    ('geometry.py', 'legacy_yaw_from_quaternion'),
    ('geometry.py', 'transform_pose'),
    ('grid.py', '<module>'),
    ('grid.py', 'GridMapView'),
    ('grid.py', 'GridMapView.in_bounds'),
    ('grid.py', 'GridMapView.value'),
    ('grid.py', 'GridMapView.is_goal_free'),
    ('grid.py', 'GridMapView.is_blocked'),
    ('grid.py', 'GridMapView.world_to_cell'),
    ('grid.py', 'GridMapView.cell_center_world'),
    ('grid.py', 'GridMapView.from_occupancy_grid'),
    ('footprint.py', '<module>'),
    ('footprint.py', 'OccupancyGridView'),
    ('footprint.py', 'OccupancyGridView.from_message'),
    ('footprint.py', 'FootprintCheck'),
    ('footprint.py', 'parse_footprint'),
    ('footprint.py', 'check_footprint'),
])
def test_reviewed_geometry_summary(path, qualified_name):
    """只检查指定说明的中文摘要，不把语言存在性当语义正确证明.

    Args:
        path: 领域模块文件名。
        qualified_name: 已审模块、类或方法的限定名。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks/domain'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef))
                       and child.name == name]
            assert len(matches) == 1
            node = matches[0]
    summary = (ast.get_docstring(node) or '').splitlines()[0]
    assert re.search('[\u4e00-\u9fff]', summary), qualified_name


def _grid_message():
    """生成纯内存旋转栅格消息，不创建 ROS 节点或真实地图."""
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='world'),
        info=SimpleNamespace(
            width=3, height=3, resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=10.0, y=10.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)))),
        data=[-1] * 9)


def test_grid_models_have_distinct_rotation_and_unknown_policies():
    """Dock 轮廓模型处理原点旋转/允许未知，旧目标格模型不作同等承诺."""
    message = _grid_message()
    legacy = GridMapView.from_occupancy_grid(message)
    dock = OccupancyGridView.from_message(message)
    polygon = ((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2))
    assert legacy.world_to_cell(8.0, 12.0) is None
    assert not legacy.is_goal_free(2, 2)
    assert legacy.is_blocked(2, 2, 50, True)
    assert not legacy.is_blocked(2, 2, 50, False)
    result = check_footprint(dock, polygon, (8.0, 12.0, 0.0))
    assert result.clear and result.reason == 'clear'


def test_grid_factories_copy_data_without_full_validation():
    """工厂复制消息数据，但不把尺寸/分辨率解析等同完整合法性校验."""
    message = _grid_message()
    message.info.resolution = 0.0
    message.data = [75]
    legacy = GridMapView.from_occupancy_grid(message)
    dock = OccupancyGridView.from_message(message)
    message.data[0] = 0
    assert legacy.cells == [75]
    assert dock.data == (75,)
    legacy.cells[0] = 50
    assert dock.data == (75,)
    with pytest.raises(ZeroDivisionError):
        legacy.world_to_cell(10.0, 10.0)
    result = check_footprint(dock, ((0, 0), (1, 0), (0, 1)), (0, 0, 0))
    assert not result.clear and result.reason == 'invalid_static_map'


def test_occupied_cell_boundary_contact_counts_as_collision():
    """轮廓只接触占用格边界也拒绝，不能只检查点或轮廓中心."""
    data = tuple(50 if index == 6 else 0 for index in range(16))
    grid = OccupancyGridView('world', 4, 4, 1.0, 0.0, 0.0, 0.0, data)
    polygon = ((-0.5, -0.4), (0.5, -0.4), (0.5, 0.4), (-0.5, 0.4))
    result = check_footprint(grid, polygon, (1.5, 1.5, 0.0))
    assert not result.clear
    assert result.reason == 'occupied_static_cell'
    assert result.occupied_cell == (2, 1, 50)


def test_footprint_parser_accepts_literals_not_general_yaml():
    """保留 Python 字面量解析契约，不宣传为通用 YAML 解析器."""
    assert parse_footprint('((0, 0), (1, 0), (0, 1))') == (
        (0.0, 0.0), (1.0, 0.0), (0.0, 1.0))
    with pytest.raises(ValueError):
        parse_footprint('- [0, 0]\n- [1, 0]\n- [0, 1]')
    with pytest.raises(ValueError):
        parse_footprint('[[False, 0], [1, 0], [0, 1]]')


def test_transform_retains_unvalidated_timestamp():
    """坐标变换验证几何，但不会把非有限时间戳改写或拒绝."""
    source = PoseRecord('caller_owned_frame', 1.0, 2.0, 3.0, 0.0, float('nan'))
    result = transform_pose(Transform2D(0.0, 0.0, 0.0), source, target_frame='world')
    assert (result.x, result.y, result.z) == (1.0, 2.0, 3.0)
    assert math.isnan(result.stamp_sec)


def test_legacy_yaw_keeps_zero_and_positive_pi_behavior():
    """迁移辅助函数保留零四元数和正 pi，不等同归一化版本."""
    assert legacy_yaw_from_quaternion(0.0, 0.0, 0.0, 0.0) == 0.0
    assert legacy_yaw_from_quaternion(0.0, 0.0, 1.0, 0.0) == math.pi
