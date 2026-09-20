"""本批已审说明对应的纯数据边界：不启动节点、不创建目录、不接触 ROS."""

import ast
from pathlib import Path
import re

import pytest

import nav_bringup
from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.composition import component_specs
from nav_bringup.launching.graph import build_graph, exit_decision, GraphOptions, ProcessSpec

MODULES = ('launching/graph.py', 'launching/component_loader.py', 'launching/composition.py',
           'launching/executables.py', 'launching/ready_exec.py', 'launching/bundle.py',
           'launching/supervision.py')


@pytest.mark.parametrize('name', MODULES)
def test_reviewed_summaries_are_chinese(name):
    """检查本批模块的现有摘要，不声称语言存在性可以证明说明语义."""
    path = Path(nav_bringup.__file__).parent / name
    tree = ast.parse(path.read_text(encoding='utf-8'))
    missing = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node)
            if doc and re.search(r'[一-鿿]', doc.splitlines()[0]) is None:
                missing.append((getattr(node, 'lineno', 1), getattr(node, 'name', '<module>')))
    assert not missing, missing


def _process(**changes):
    """构造一个仅用于决策的进程约定，不解析可执行文件、不启动任何东西."""
    values = dict(name='driver', package='fixture', executable='fixture', parameter_file='',
                  remappings=(), arguments=(), delay_sec=0.0, restart_limit=2, critical=True,
                  wait_for_navigation=False, ros_node=True, ready_timeout_sec=1.0,
                  ready_action='navigate_to_pose', configuration_files=())
    values.update(changes)
    return ProcessSpec(**values)


def test_shutdown_outranks_a_remaining_restart_budget():
    """关停期即使预算没用完也不重启；此时的退出属于预期，按忽略处理."""
    process = _process(restart_limit=2)
    assert exit_decision(process, 0, False) == 'restart'
    assert exit_decision(process, 0, True) == 'ignore'
    assert exit_decision(process, 2, False) == 'shutdown'


def test_noncritical_exit_never_stops_the_stack():
    """非关键进程预算用尽也只被忽略，不会把整栈拖停；关键进程才关停."""
    assert exit_decision(_process(name='rviz', restart_limit=0, critical=False), 0, False) \
        == 'ignore'
    assert exit_decision(_process(restart_limit=0), 0, False) == 'shutdown'


@pytest.mark.parametrize('prior', [-1, True, 1.0, '1'])
def test_invalid_restart_counter_is_refused(prior):
    """重启计数非法直接拒绝，不按零处理、不猜测."""
    with pytest.raises(ValueError):
        exit_decision(_process(), prior, False)


def test_relative_run_directory_is_refused(tmp_path, monkeypatch):
    """运行目录必须是绝对路径；相对路径不会被解析成当前工作目录."""
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError):
        load_options('.', {})


def test_no_components_are_selected_without_explicit_composition():
    """未显式要求组合时不挑任何组件，节点保持各自独立进程."""
    graph = build_graph(GraphOptions('tour', '/run/test', start_executor=True))
    assert not graph.composition
    assert component_specs(graph) == ()
