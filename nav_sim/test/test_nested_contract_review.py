"""核验自测嵌套契约与启动门禁，所有端口均为内存替身."""

import ast
import asyncio
from pathlib import Path

import pytest

from nav_sim.selftest_coordinator import drive
from test_selftest_coordinator import _ports


@pytest.mark.parametrize('path, qualified_name', [
    ('fake_agent_server.py', '_serve.handler'),
    ('selftest_coordinator.py', 'drive.until'),
    ('selftest_coordinator.py', 'drive.arrived'),
])
def test_nested_selftest_contract_is_documented(path, qualified_name):
    """按作用域名称检查嵌套说明，包含循环体中的局部函数.

    Args:
        path: 包内源文件名。
        qualified_name: 按父作用域限定的函数名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_sim'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in ast.walk(node)
                   if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
                   and child is not node and child.name == name]
        assert len(matches) == 1
        node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc and any('一' <= char <= '鿿' for char in doc.splitlines()[0])


@pytest.mark.parametrize('fault', ['peer', 'severity'])
def test_ready_startup_still_rejects_unhealthy_evidence(tmp_path, fault):
    """只读就绪不能掩盖连接错误或不健康任务事件，且始终不发送目标.

    Args:
        tmp_path: 测试私有证据目录。
        fault: 注入连接错误或任务事件严重程度错误。
    """
    scene, trace, peer, observer, child, state = _ports(tmp_path)
    if fault == 'peer':
        peer.error = 'connection_failed'
    else:
        trace.nav.append({'source': 'tasks', 'event_type': 'task_admitted',
                          'severity': 'warn'})
    with trace, pytest.raises(RuntimeError, match=(
            'connection_failed' if fault == 'peer' else 'unhealthy task observation')):
        asyncio.run(drive(scene, trace, peer, observer, child, 1, True))
    assert state.sent == []
