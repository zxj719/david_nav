"""保护已审任务端口、目标解析与技能的中文摘要，不替代语义或实机验证."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('core/ports.py', '<module>'),
    ('core/ports.py', 'DockBackend'),
    ('core/ports.py', 'DockBackend.dock'),
    ('core/ports.py', 'NavigationBackend'),
    ('core/ports.py', 'NavigationBackend.navigate'),
    ('core/ports.py', 'NavigationBackend.cancel'),
    ('core/ports.py', 'SpinBackend'),
    ('core/ports.py', 'SpinBackend.spin'),
    ('core/ports.py', 'SpinBackend.cancel'),
    ('core/ports.py', 'PoseSource'),
    ('core/ports.py', 'PoseSource.current_map_pose'),
    ('core/ports.py', 'PoseSource.capture_for_poi'),
    ('core/ports.py', 'PoseSource.current_map_from_odom'),
    ('core/ports.py', 'PoseSource.current_slam_session_id'),
    ('core/ports.py', 'PoiRepository'),
    ('core/ports.py', 'PoiRepository.load'),
    ('core/ports.py', 'PoiRepository.get'),
    ('core/ports.py', 'PoiRepository.upsert'),
    ('core/ports.py', 'PoiRepository.rename_many'),
    ('core/ports.py', 'PoiRepository.prune_session'),
    ('core/ports.py', 'PoiRepository.reproject_session'),
    ('core/ports.py', 'GoalResolver'),
    ('core/ports.py', 'GoalResolver.resolve'),
    ('core/ports.py', 'GoalResolver.is_admissible'),
    ('core/ports.py', 'Clock'),
    ('core/ports.py', 'Clock.now'),
    ('core/ports.py', 'Clock.sleep'),
    ('core/goal_resolution.py', '<module>'),
    ('core/goal_resolution.py', 'SnapSearch'),
    ('core/goal_resolution.py', 'SnapSearch.__post_init__'),
    ('core/goal_resolution.py', 'SnapSearch.resolve'),
    ('core/goal_resolution.py', 'SnapSearch.is_admissible'),
    ('core/goal_resolution.py', 'DirectGoalResolver'),
    ('core/goal_resolution.py', 'DirectGoalResolver.resolve'),
    ('core/goal_resolution.py', 'DirectGoalResolver.is_admissible'),
    ('core/goal_resolution.py', 'GridGoalResolver'),
    ('core/goal_resolution.py', 'GridGoalResolver.__init__'),
    ('core/goal_resolution.py', 'GridGoalResolver.resolve'),
    ('core/goal_resolution.py', 'GridGoalResolver.is_admissible'),
    ('core/handlers/navigate_to.py', '<module>'),
    ('core/handlers/navigate_to.py', 'NavigateToPoiRequest'),
    ('core/handlers/navigate_to.py', 'NavigateToPoi'),
    ('core/handlers/navigate_to.py', 'NavigateToPoi.__init__'),
    ('core/handlers/navigate_to.py', 'NavigateToPoi.decode'),
    ('core/handlers/navigate_to.py', 'NavigateToPoi.run'),
    ('core/handlers/dock.py', '<module>'),
    ('core/handlers/dock.py', 'DockRequest'),
    ('core/handlers/dock.py', 'Dock'),
    ('core/handlers/dock.py', 'Dock.__init__'),
    ('core/handlers/dock.py', 'Dock.decode'),
    ('core/handlers/dock.py', 'Dock.run'),
    ('core/handlers/dock.py', 'DockAwareNavigation'),
    ('core/handlers/dock.py', 'DockAwareNavigation.__init__'),
    ('core/handlers/dock.py', 'DockAwareNavigation.decode'),
    ('core/handlers/dock.py', 'DockAwareNavigation.run'),
])
def test_reviewed_ports_and_handlers_summary(path, qualified_name):
    """逐项检查已审摘要，行为仍由对应实现和既有特征测试证明.

    Args:
        path: 包内模块相对路径。
        qualified_name: 模块、类或方法的限定名。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                       and child.name == name]
            assert len(matches) == 1
            node = matches[0]
    summary = (ast.get_docstring(node) or '').splitlines()[0]
    assert re.search('[\u4e00-\u9fff]', summary), qualified_name
