"""保护已审查的中文摘要与原有帮助输出；语义正确性仍依赖代码审查."""

import ast
import importlib
from pathlib import Path
import re
import sys

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('__init__.py', '<module>'),
    ('_ros.py', '<module>'),
    ('_ros.py', 'require_node_sandbox'),
    ('_ros.py', 'run'),
    ('fake_agent_server.py', '<module>'),
    ('fake_agent_server.py', 'navigate_to'),
    ('fake_agent_server.py', 'abort'),
    ('fake_agent_server.py', 'ScriptStep'),
    ('fake_agent_server.py', 'load_script'),
    ('fake_agent_server.py', 'AgentSession'),
    ('fake_agent_server.py', 'AgentSession.run'),
    ('fake_agent_server.py', 'main'),
    ('fake_chassis.py', '<module>'),
    ('fake_chassis.py', 'FakeChassis'),
    ('fake_chassis.py', 'FakeChassis.__init__'),
    ('fake_chassis.py', 'main'),
    ('fake_localization.py', '<module>'),
    ('fake_localization.py', 'FakeLocalization'),
    ('fake_localization.py', 'FakeLocalization.__init__'),
    ('fake_localization.py', 'main'),
    ('model.py', '<module>'),
    ('model.py', 'PlanarPose'),
    ('model.py', 'finite_number'),
    ('model.py', 'integrate'),
    ('model.py', 'quaternion'),
    ('model.py', 'require_sandbox'),
    ('model.py', 'frame_name'),
    ('model.py', 'localization_frames'),
    ('selftest.py', '<module>'),
    ('selftest.py', 'run'),
    ('selftest_coordinator.py', '<module>'),
    ('selftest_coordinator.py', 'Peer'),
    ('selftest_coordinator.py', 'Peer.__init__'),
    ('selftest_coordinator.py', 'Peer.handle'),
    ('selftest_coordinator.py', 'drive'),
    ('selftest_evidence.py', '<module>'),
    ('selftest_evidence.py', 'ExpectedCase'),
    ('selftest_evidence.py', 'EvidenceResult'),
    ('selftest_evidence.py', 'check_evidence'),
    ('selftest_launch.py', '<module>'),
    ('selftest_launch.py', 'close_fixture'),
    ('selftest_launch.py', 'main'),
    ('selftest_plan.py', '<module>'),
    ('selftest_plan.py', 'load_scene'),
    ('selftest_ros.py', '<module>'),
    ('selftest_ros.py', 'Observer'),
    ('selftest_ros.py', 'Observer.__init__'),
    ('selftest_ros.py', 'Observer.step'),
    ('selftest_ros.py', 'Observer.pose'),
    ('selftest_ros.py', 'Observer.odometry'),
    ('selftest_ros.py', 'Observer.ready'),
    ('selftest_ros.py', 'Observer.close'),
    ('selftest_runtime.py', '<module>'),
    ('selftest_runtime.py', 'require_idle_host'),
    ('selftest_runtime.py', 'execute'),
    ('selftest_scene.py', '<module>'),
    ('selftest_scene.py', 'Scene'),
    ('selftest_scene.py', 'prepare_scene'),
    ('selftest_trace.py', '<module>'),
    ('selftest_trace.py', 'EvidenceTrace'),
    ('selftest_trace.py', 'EvidenceTrace.__init__'),
    ('selftest_trace.py', 'EvidenceTrace.receive'),
    ('selftest_trace.py', 'EvidenceTrace.close'),
    ('selftest_trace.py', 'EvidenceTrace.__enter__'),
    ('selftest_trace.py', 'EvidenceTrace.__exit__'),
    ('selftest_trace.py', 'reached'),
    ('selftest_trace.py', 'audit_processes'),
])
def test_reviewed_documentation_has_chinese_summary(path, qualified_name):
    """逐名检查中文首句，不能据此声称文档或运行行为已完整验收.

    Args:
        path: 包内模块文件名。
        qualified_name: 模块标记或类、函数的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_sim'
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


@pytest.mark.parametrize('name, program', [
    ('fake_agent_server', 'fake_agent_server'),
    ('selftest_launch', 'selftest_launch.py'),
])
def test_help_keeps_original_output_without_runtime(name, program, monkeypatch, capsys):
    """帮助必须逐字不变，且在域检查前退出而不进入服务或启动流程.

    Args:
        name: 可单独显示帮助的模块名。
        program: 与安装入口或模块运行对应的稳定程序名。
        monkeypatch: pytest 环境和属性替换器。
        capsys: pytest 标准输出捕获器。
    """
    module = importlib.import_module('nav_sim.' + name)
    monkeypatch.setenv('COLUMNS', '80')
    monkeypatch.setattr(sys, 'argv', [program])
    monkeypatch.setattr(module, 'require_sandbox',
                        lambda *_: pytest.fail('help must exit before runtime guards'))
    with pytest.raises(SystemExit) as result:
        module.main(['--help'])
    assert result.value.code == 0
    output = capsys.readouterr()
    expected = (Path(__file__).parent / 'golden' / (name + '_help.txt')).read_text(
        encoding='utf-8')
    assert output.out == expected
    assert output.err == ''
