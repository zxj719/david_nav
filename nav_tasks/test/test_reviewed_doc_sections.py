"""检查本轮审查过的函数分节和参数描述，不替代语义审查."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name, sections', [
    ('domain/footprint.py', '_valid_polygon', ['Args', 'Returns']),
    ('domain/footprint.py', '_pose_polygon', ['Args', 'Returns']),
    ('domain/footprint.py', '_world_to_grid_local', ['Args', 'Returns']),
    ('domain/footprint.py', '_polygons_intersect', ['Args', 'Returns']),
    ('domain/footprint.py', '_point_in_polygon', ['Args', 'Returns']),
    ('domain/footprint.py', '_segments_intersect', ['Args', 'Returns']),
    ('adapters/ros/dock_motion.py', '_DockMotion._dispatch', ['Args', 'Raises']),
])
def test_reviewed_function_sections(path, qualified_name, sections):
    """按限定名称检查中文说明、必要分节和每个参数名.

    异常是否被内部捕获由逐项控制流审查决定，不能按出现 raise 自动推断。
    这里只保护已审说明结构，不证明文字与实现语义一致。

    Args:
        path: 包内源码的相对路径。
        qualified_name: 包含外层类或函数名的限定名称。
        sections: 本次审查确认必需的 Google 分节名。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks'
    tree = ast.parse((root / path).read_text(encoding='utf-8'))
    node = tree
    for name in qualified_name.split('.'):
        pending = list(ast.iter_child_nodes(node))
        matches = []
        while pending:
            current = pending.pop()
            if isinstance(current, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                if current.name == name:
                    matches.append(current)
            else:
                pending.extend(ast.iter_child_nodes(current))
        assert len(matches) == 1, qualified_name
        node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert re.search('[\u4e00-\u9fff]', doc), qualified_name
    for section in sections:
        assert re.search(r'^' + section + r':\s*$', doc, re.MULTILINE), qualified_name
    parameters = node.args.posonlyargs + node.args.args + node.args.kwonlyargs
    parameters += [arg for arg in (node.args.vararg, node.args.kwarg) if arg is not None]
    if 'Args' in sections:
        args_text = doc.split('Args:', 1)[1].split('\n\n', 1)[0]
        for arg in parameters:
            if arg.arg not in ('self', 'cls'):
                assert re.search(r'^\s+\**' + re.escape(arg.arg) + r':',
                                 args_text, re.MULTILINE), arg.arg
