"""短函数中的适配边界说明不得因实现简短而遗漏."""

import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('control.py', '_lift'),
])
def test_nonobvious_private_contract_has_documentation(path, qualified_name):
    """按限定名保留说明要求，文字语义仍需逐项审查.

    Args:
        path: 相对于包源码的模块路径。
        qualified_name: 包含所属类名的函数限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'chassis_teleop'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in node.body
                   if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                   and child.name == name]
        assert len(matches) == 1, qualified_name
        node = matches[0]
    assert isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)), qualified_name
    assert (ast.get_docstring(node) or '').strip(), qualified_name
