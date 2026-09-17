"""保护非平凡嵌套函数说明，不执行诊断、信号、启动或删除."""

import ast
from pathlib import Path

import pytest

from nav_bringup.navcfg.live_inputs import parameter_sections


@pytest.mark.parametrize('path, qualified_name', [
    ('doctor/live.py', 'ros_transform_probe.probe'),
    ('launching/runtime.py', 'launch_actions.factory'),
    ('navcfg/live_inputs.py', 'parameter_sections.flatten'),
    ('navcfg/live_inputs.py', 'parameter_sections.walk'),
    ('navrun/preflight.py', 'cleanup_preflight.send'),
    ('navrun/record_reader.py', 'RecordReader.tree.visit'),
    ('navrun/runtime_host.py', 'bind_host_steps.platform'),
    ('navrun/runtime_host.py', 'bind_host_steps.preflight'),
    ('navrun/runtime_host.py', 'bind_host_steps.dds'),
    ('recorder/safe_retention.py', 'inspect_at.walk'),
])
def test_nested_contract_summary(path, qualified_name):
    """检查父作用域限定的嵌套说明，语言存在性不替代语义.

    Args:
        path: 包内模块相对路径。
        qualified_name: 函数或方法限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_bringup'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in ast.walk(node)
                   if isinstance(child, (ast.ClassDef, ast.FunctionDef))
                   and child is not node and child.name == name]
        assert len(matches) == 1
        node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc and any('一' <= char <= '鿿' for char in doc.splitlines()[0])


def test_parameter_flattening_preserves_leaf_references():
    """展开只重建字典和参数键，列表等叶子仍是调用方原对象."""
    values = [1, 2]
    tree = {'scope': {'node': {'ros__parameters': {'nested': {'values': values}}}}}
    result = parameter_sections(tree)
    assert result == {'/scope/node': {'nested.values': [1, 2]}}
    assert result['/scope/node']['nested.values'] is values
    values.append(3)
    assert result['/scope/node']['nested.values'] == [1, 2, 3]
