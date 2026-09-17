"""短小但非直观的清理与输入边界函数必须保留说明."""

import ast
from pathlib import Path

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('adapters/ros/dock_ingress.py', 'DockIngress._log_exception'),
    ('adapters/ros/grid_source.py', 'RosGridSource._receive'),
    ('core/state.py', 'TaskManager._is_current'),
    ('core/state.py', 'TaskManager._starting'),
    ('core/state.py', 'TaskManager._describe'),
    ('core/state.py', 'TaskManager._feedback'),
    ('core/state.py', 'TaskManager._emit'),
    ('domain/footprint.py', '_cell_polygon'),
    ('skills/docking/finish.py', '_wrap'),
    ('skills/docking/finish.py', '_errors'),
    ('skills/docking/settle.py', '_wrap'),
    ('adapters/ros/dock_ingress.py', 'DockIngress._pose'),
    ('adapters/ros/dock_ingress.py', 'DockIngress._topic_callback'),
    ('adapters/ros/dock_ingress.py', 'DockIngress._goal_callback'),
    ('adapters/ros/dock_ingress.py', 'DockIngress._observe'),
    ('adapters/ros/dock_motion.py', '_DockMotion._call'),
    ('adapters/ros/dock_motion.py', '_DockMotion._finish_stage'),
    ('adapters/ros/dock_observations.py', 'DockObservations._receive'),
    ('adapters/ros/tf_pose_source.py', 'TfPoseSource._lookup'),
    ('nav2_client.py', 'Nav2Client._rollback_clients'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository._reproject_pose'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository._stored_poi_from_mapping'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository._pose_from_mapping'),
    ('skills/docking/workflow.py', 'DockWorkflow._observe'),
    ('skills/docking/workflow.py', 'DockWorkflow._report'),
    ('adapters/ros/action_session.py', 'ActionSession._cancel_attempt'),
    ('adapters/ros/action_session.py', 'ActionSession._queue_feedback'),
    ('adapters/ros/action_session.py', 'ActionSession._feedback'),
    ('adapters/ros/dock_ingress.py', 'DockIngress._cancel_owned'),
    ('adapters/ros/dock_ingress.py', 'DockIngress._close'),
    ('adapters/ros/dock_worker.py', 'DockWorker._cleanup_task'),
    ('adapters/ros/dock_worker.py', 'DockWorker._cleanup'),
    ('adapters/ros/dock_worker.py', 'DockWorker._wait_thread'),
    ('app/runtime.py', 'TaskRuntime._settle'),
    ('app/runtime.py', 'TaskRuntime._close'),
    ('core/state.py', 'TaskManager._cancel'),
    ('core/state.py', 'TaskManager._close'),
    ('core/state.py', 'TaskManager._recover_crash'),
])
def test_nonobvious_private_contract_has_documentation(path, qualified_name):
    """按稳定名称查找函数，避免缩短实现后绕过长函数说明门禁.

    仅检查说明存在；取消、线程与校验语义仍需人工逐项审查。

    Args:
        path: 相对于包源码目录的模块路径。
        qualified_name: 类与函数组成的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    for name in qualified_name.split('.'):
        matches = [child for child in node.body
                   if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                   and child.name == name]
        assert len(matches) == 1, qualified_name
        node = matches[0]
    assert isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)), qualified_name
    assert (ast.get_docstring(node) or '').strip(), qualified_name
