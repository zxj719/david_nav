"""保护本轮审查的遥控中文摘要，不把字符检查当作语义证明."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('__init__.py', '<module>'),
    ('control.py', '<module>'),
    ('control.py', 'TeleopConfig'),
    ('control.py', 'TeleopConfig.validate'),
    ('control.py', 'TeleopConfig.period_sec'),
    ('control.py', 'shape_motion'),
    ('control.py', 'motion_for_key'),
    ('control.py', 'KeyboardState'),
    ('control.py', 'KeyboardState.__init__'),
    ('control.py', 'KeyboardState.handle_key'),
    ('control.py', 'KeyboardState.command'),
    ('interlock.py', '<module>'),
    ('interlock.py', 'CommandConflict'),
    ('interlock.py', 'wait_for_clear_topics'),
    ('node.py', '<module>'),
    ('node.py', 'KeyboardNode'),
    ('node.py', 'KeyboardNode.__init__'),
    ('node.py', 'KeyboardNode.arm'),
    ('node.py', 'KeyboardNode.handle_key'),
    ('node.py', 'KeyboardNode.publish_current_command'),
    ('node.py', 'KeyboardNode.publish_zero'),
    ('node.py', 'KeyboardNode.publish_stop_burst'),
    ('node.py', 'run_keyboard'),
    ('node.py', 'main'),
    ('node.py', 'managed_main'),
    ('terminal.py', '<module>'),
    ('terminal.py', 'TerminalUnavailable'),
    ('terminal.py', 'TerminalKeyboard'),
    ('terminal.py', 'TerminalKeyboard.__init__'),
    ('terminal.py', 'TerminalKeyboard.__enter__'),
    ('terminal.py', 'TerminalKeyboard.__exit__'),
    ('terminal.py', 'TerminalKeyboard.fileno'),
    ('terminal.py', 'TerminalKeyboard.close'),
    ('terminal.py', 'TerminalKeyboard.read_key'),
])
def test_reviewed_documentation_has_chinese_summary(path, qualified_name):
    """按限定名称检查模块、类和函数的中文摘要.

    Args:
        path: 包内源码文件名。
        qualified_name: 模块标记或类与函数的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'chassis_teleop'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                       and child.name == name]
            assert len(matches) == 1, qualified_name
            node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc.strip(), qualified_name
    assert re.search('[\u4e00-\u9fff]', doc.splitlines()[0]), qualified_name
