"""保护短小但非直观的进程与文件安全边界说明."""

import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('navrun/dds_hygiene.py', '_pid_entries'),
    ('navrun/dds_hygiene.py', '_directory'),
    ('navrun/owned_process.py', 'OwnedProcess._check_owner'),
    ('navrun/owned_process.py', 'OwnedProcess._signal'),
    ('navrun/preparation.py', '_publish_marker'),
    ('navrun/record_reader.py', 'RecordReader._directory'),
    ('navrun/recorded_startup.py', '_check_plan'),
    ('navrun/recorded_startup.py', '_write_manifest'),
    ('navrun/recorded_startup.py', '_gc'),
    ('navrun/wrap.py', '_SignalRelay._handle'),
    ('recorder/linked_process.py', '_unique_target'),
    ('recorder/nav_events.py', '_pairs'),
    ('recorder/retention.py', '_require_run_under'),
    ('recorder/sessions.py', '_signal_group'),
])
def test_nonobvious_private_contract_has_documentation(path, qualified_name):
    """按名称保留说明要求，不把函数长度或说明存在当作安全证明.

    Args:
        path: 相对于包源码目录的模块路径。
        qualified_name: 类与函数组成的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_bringup'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in node.body
                   if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                   and child.name == name]
        assert len(matches) == 1, qualified_name
        node = matches[0]
    assert isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)), qualified_name
    assert (ast.get_docstring(node) or '').strip(), qualified_name
