"""保护已审查的适配器说明使用中文摘要，不替代契约语义审查."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('__init__.py', '<module>'),
    ('core.py', '<module>'),
    ('core.py', 'AdapterConfig'),
    ('core.py', 'AdapterConfig.validate'),
    ('core.py', 'AdapterConfig.period_sec'),
    ('core.py', 'shape_command'),
    ('core.py', 'AdapterState'),
    ('core.py', 'AdapterState.__init__'),
    ('core.py', 'AdapterState.receive'),
    ('core.py', 'AdapterState.is_stale'),
    ('core.py', 'AdapterState.output'),
    ('core.py', 'AdapterState.diagnostics'),
    ('node.py', '<module>'),
    ('node.py', 'AdapterNode'),
    ('node.py', 'AdapterNode.__init__'),
    ('node.py', 'AdapterNode.velocity_callback'),
    ('node.py', 'AdapterNode.publish_current_command'),
    ('node.py', 'AdapterNode.publish_diagnostics'),
    ('node.py', 'AdapterNode.publish_zero'),
    ('node.py', 'AdapterNode.publish_stop_burst'),
    ('node.py', 'main'),
])
def test_reviewed_documentation_has_chinese_summary(path, qualified_name):
    """按稳定名称检查中文摘要，防止混合说明被整个字符串的 CJK 命中掩盖.

    Args:
        path: 适配器包源码内的相对文件名。
        qualified_name: 模块标记或类与函数的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'chassis_adapter'
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
