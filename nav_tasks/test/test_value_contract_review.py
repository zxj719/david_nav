"""保护已审领域值对象摘要和构造边界，不替代运行时或实机校验."""

import ast
import asyncio
from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import re

import pytest

from nav_tasks.domain.errors import RuntimeAssemblyError
from nav_tasks.domain.runtime_config import RuntimeConfig
from nav_tasks.domain.tasks import TaskRequest
from nav_tasks.domain.types import PoseRecord


@pytest.mark.parametrize('path, qualified_name', [
    ('__init__.py', '<module>'),
    ('domain/__init__.py', '<module>'),
    ('domain/errors.py', '<module>'),
    ('domain/errors.py', 'NavTaskError'),
    ('domain/errors.py', 'PoseUnavailableError'),
    ('domain/errors.py', 'PoiNotFoundError'),
    ('domain/errors.py', 'ConfigError'),
    ('domain/errors.py', 'RuntimeAssemblyError'),
    ('domain/errors.py', 'RuntimeAssemblyError.__init__'),
    ('domain/errors.py', 'BusyError'),
    ('domain/errors.py', 'ClosedError'),
    ('domain/errors.py', 'UnknownActionError'),
    ('domain/runtime_config.py', '<module>'),
    ('domain/runtime_config.py', 'DockRuntimeConfig'),
    ('domain/runtime_config.py', 'DockNodeConfig'),
    ('domain/runtime_config.py', 'RuntimeConfig'),
    ('domain/types.py', '<module>'),
    ('domain/types.py', 'PoseRecord'),
    ('domain/types.py', 'PoseRecord.orientation'),
    ('domain/types.py', 'StoredPoi'),
    ('domain/types.py', 'Transform2D'),
    ('domain/types.py', 'CapturedPose'),
    ('domain/tasks.py', '<module>'),
    ('domain/tasks.py', 'NavStatus'),
    ('domain/tasks.py', 'NavOutcome'),
    ('domain/tasks.py', 'NavOutcome.succeeded'),
    ('domain/tasks.py', 'NavGoal'),
    ('domain/tasks.py', 'TaskCompletion'),
    ('domain/tasks.py', 'SpinGoal'),
    ('domain/tasks.py', 'DockGoal'),
    ('domain/tasks.py', 'ResolvedGoal'),
    ('domain/tasks.py', 'PreemptPolicy'),
    ('domain/tasks.py', 'TaskState'),
    ('domain/tasks.py', 'TaskRequest'),
    ('domain/tasks.py', 'TaskSnapshot'),
    ('domain/tasks.py', 'EventKind'),
    ('domain/tasks.py', 'TaskEvent'),
])
def test_reviewed_value_summary(path, qualified_name):
    """只保护已审中文摘要，不声称静态文字检查能证明运行语义.

    Args:
        path: 包内源码相对路径。
        qualified_name: 模块、类或方法的限定名。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks'
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


def test_frozen_request_does_not_copy_or_freeze_payload():
    """冻结请求的字段引用，但构造器不会复制或深冻结外部载荷."""
    payload = {'target_id': 'before'}
    request = TaskRequest('request', 'navigate_to', payload)
    payload['target_id'] = 'after'
    assert request.payload is payload
    assert request.payload['target_id'] == 'after'
    with pytest.raises(FrozenInstanceError):
        request.task_id = 'replacement'


def test_assembly_error_retains_callback_without_invoking_or_setting_cause():
    """异常构造只保留清理回调；调用和异常链分别由宿主管理."""
    calls = []

    async def cleanup():
        calls.append('cleanup')

    original = OSError('fixture')
    error = RuntimeAssemblyError(original, cleanup)
    assert error.cleanup is cleanup
    assert error.__cause__ is None
    assert str(original) in str(error)
    assert calls == []
    asyncio.run(error.cleanup())
    assert calls == ['cleanup']
    try:
        raise error from original
    except RuntimeAssemblyError as chained:
        assert chained.__cause__ is original


def test_runtime_values_do_not_validate_dock_assembly_on_construction():
    """值对象允许保存不完整 Dock 接线；拒绝发生在运行时工厂而非构造器."""
    config = RuntimeConfig(
        poi_store_file=Path('memory-only.yaml'), robot_base_frame='body',
        odom_topic='/odom', map_frame='world', odom_frame='odom',
        grid_map_topic='/map', global_costmap_topic='/costmap',
        action_name='/navigate', slam_session_id='fixture', dock_poi_ids=('dock',))
    assert config.dock_poi_ids == ('dock',)
    assert config.dock is None


def test_pose_orientation_does_not_add_finite_input_validation():
    """旧值对象直接转换 yaw，不具有 geometry 辅助函数的有限性门禁."""
    pose = PoseRecord('world', 0.0, 0.0, 0.0, float('nan'), 0.0)
    orientation = pose.orientation()
    assert list(orientation) == ['x', 'y', 'z', 'w']
    assert orientation['x'] == orientation['y'] == 0.0
    assert math.isnan(orientation['z']) and math.isnan(orientation['w'])
