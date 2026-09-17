"""保护已审公共 API 与内核中文摘要，不替代实现、故障或实机验证."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('api.py', '<module>'),
    ('api.py', 'TaskApi'),
    ('api.py', 'TaskApi.submit'),
    ('api.py', 'TaskApi.cancel'),
    ('api.py', 'TaskApi.status'),
    ('api.py', 'TaskApi.subscribe_events'),
    ('api.py', 'TaskApi.unsubscribe_events'),
    ('api.py', 'TaskApi.close'),
    ('api.py', 'DockIngressApi'),
    ('api.py', 'DockIngressApi.open'),
    ('api.py', 'DockIngressApi.stop'),
    ('api.py', 'DockIngressApi.close'),
    ('api.py', 'DockIngressApi.destroy'),
    ('api.py', 'DockIngressApi.on_feedback'),
    ('api.py', 'create_dock_ingress'),
    ('api.py', 'PoiApi'),
    ('api.py', 'PoiApi.mark'),
    ('api.py', 'PoiApi.update_directory'),
    ('api.py', 'TaskRuntimeApi'),
    ('api.py', 'TaskRuntimeApi.start_loop_closure'),
    ('api.py', 'TaskRuntimeApi.stop_loop_closure'),
    ('api.py', 'TaskRuntimeApi.settle'),
    ('api.py', 'TaskRuntimeApi.close'),
    ('api.py', 'dock_config_from_parameters'),
    ('api.py', 'declare_dock_parameters'),
    ('api.py', 'create_runtime'),
    ('core/__init__.py', '<module>'),
    ('core/registry.py', '<module>'),
    ('core/registry.py', 'TaskContext'),
    ('core/registry.py', 'TaskContext.check_cancelled'),
    ('core/registry.py', 'CommandRegistry'),
    ('core/registry.py', 'CommandRegistry.__init__'),
    ('core/registry.py', 'CommandRegistry.register'),
    ('core/registry.py', 'CommandRegistry.prepare'),
    ('core/state.py', '<module>'),
    ('core/state.py', 'TaskManager'),
    ('core/state.py', 'TaskManager.__init__'),
    ('core/state.py', 'TaskManager.submit'),
    ('core/state.py', 'TaskManager.cancel'),
    ('core/state.py', 'TaskManager.status'),
    ('core/state.py', 'TaskManager.subscribe_events'),
    ('core/state.py', 'TaskManager.unsubscribe_events'),
    ('core/state.py', 'TaskManager.close'),
    ('core/supervisor.py', '<module>'),
    ('core/supervisor.py', 'TaskSupervisor'),
    ('core/supervisor.py', 'TaskSupervisor.__init__'),
    ('core/supervisor.py', 'TaskSupervisor.active_count'),
    ('core/supervisor.py', 'TaskSupervisor.spawn'),
    ('core/supervisor.py', 'TaskSupervisor.cancel_and_wait'),
    ('core/retry.py', '<module>'),
    ('core/retry.py', 'MonotonicClock'),
    ('core/retry.py', 'MonotonicClock.now'),
    ('core/retry.py', 'MonotonicClock.sleep'),
    ('core/retry.py', 'RetryFailure'),
    ('core/retry.py', 'RetryPolicy'),
    ('core/retry.py', 'RetryPolicy.__post_init__'),
    ('core/retry.py', 'RetryPolicy.run'),
    ('core/handlers/__init__.py', '<module>'),
    ('core/handlers/poi_commands.py', '<module>'),
    ('core/handlers/poi_commands.py', 'MarkPoiResult'),
    ('core/handlers/poi_commands.py', 'PoiCommands'),
    ('core/handlers/poi_commands.py', 'PoiCommands.__init__'),
    ('core/handlers/poi_commands.py', 'PoiCommands.mark'),
    ('core/handlers/poi_commands.py', 'PoiCommands.update_directory'),
])
def test_reviewed_public_core_summary(path, qualified_name):
    """逐一锁定已审摘要；语义保证仍须看具体实现和行为测试.

    Args:
        path: 包内源码相对路径。
        qualified_name: 模块、类或函数的限定名。
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
