"""按名保护已审动作所有权说明，不替代行为或实机验收."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('nav2_client.py', '<module>'),
    ('nav2_client.py', 'Nav2Client'),
    ('nav2_client.py', 'Nav2Client.__init__'),
    ('nav2_client.py', 'Nav2Client.navigate'),
    ('nav2_client.py', 'Nav2Client.spin'),
    ('nav2_client.py', 'Nav2Client.navigate_pose'),
    ('nav2_client.py', 'Nav2Client.spin_action'),
    ('nav2_client.py', 'Nav2Client.cancel'),
    ('nav2_client.py', 'Nav2Client.settle'),
    ('nav2_client.py', 'Nav2Client.close'),
    ('adapters/__init__.py', '<module>'),
    ('adapters/ros/__init__.py', '<module>'),
    ('adapters/ros/action_session.py', '<module>'),
    ('adapters/ros/action_session.py', '_PhaseExpired'),
    ('adapters/ros/action_session.py', 'ActionClientLike'),
    ('adapters/ros/action_session.py', 'ActionClientLike.server_is_ready'),
    ('adapters/ros/action_session.py', 'ActionClientLike.send_goal_async'),
    ('adapters/ros/action_session.py', 'ActionState'),
    ('adapters/ros/action_session.py', 'ActionOutcome'),
    ('adapters/ros/action_session.py', 'ActionSession'),
    ('adapters/ros/action_session.py', 'ActionSession.__init__'),
    ('adapters/ros/action_session.py', 'ActionSession.busy'),
    ('adapters/ros/action_session.py', 'ActionSession.execute'),
    ('adapters/ros/action_session.py', 'ActionSession.cancel'),
    ('adapters/ros/action_session.py', 'ActionSession.close'),
    ('adapters/ros/futures.py', '<module>'),
    ('adapters/ros/futures.py', 'FutureLike'),
    ('adapters/ros/futures.py', 'FutureLike.add_done_callback'),
    ('adapters/ros/futures.py', 'FutureLike.result'),
    ('adapters/ros/futures.py', 'FutureLike.done'),
    ('adapters/ros/futures.py', 'await_rclpy_future'),
    ('adapters/ros/futures.py', 'wait_rclpy_future'),
    ('adapters/ros/action_session.py', '_Attempt'),
    ('core/handlers/dock.py', 'Dock.run.attempt'),
    ('core/handlers/navigate_to.py', 'NavigateToPoi._navigate.attempt'),
])
def test_reviewed_action_contract_summary(path, qualified_name):
    """检查已审模块、类、方法及嵌套尝试的中文摘要.

    Args:
        path: 包内模块相对路径。
        qualified_name: 文档对象的限定名称。
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
    doc = ast.get_docstring(node) or ''
    assert doc and re.search('[\\u4e00-\\u9fff]', doc.splitlines()[0]), qualified_name
