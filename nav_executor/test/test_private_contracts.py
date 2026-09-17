"""短小但非直观的清理与输入边界函数必须保留说明."""

import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('app/config.py', '_read_mapping'),
    ('app/config.py', '_executor_section'),
    ('app/idle_profile.py', '_positive'),
    ('protocol/codec.py', '_extract_request_context'),
    ('protocol/codec.py', '_text'),
    ('app/runtime.py', '_join'),
    ('app/runtime.py', '_retry'),
])
def test_nonobvious_private_contract_has_documentation(path, qualified_name):
    """按稳定名称查找函数，避免缩短实现后绕过长函数说明门禁.

    仅检查说明存在；取消、线程与校验语义仍需人工逐项审查。

    Args:
        path: 相对于包源码目录的模块路径。
        qualified_name: 类与函数组成的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_executor'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in node.body
                   if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                   and child.name == name]
        assert len(matches) == 1, qualified_name
        node = matches[0]
    assert isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)), qualified_name
    assert (ast.get_docstring(node) or '').strip(), qualified_name
