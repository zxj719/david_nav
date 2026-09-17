"""关联日志与触发器契约回归，仅使用临时文件和纯 Python 对象."""

import ast
from pathlib import Path
import re

import pytest

import nav_bringup
from nav_bringup.recorder import bt_triggers, linked_process


@pytest.mark.parametrize('name', ['linked_process.py', 'bt_triggers.py'])
def test_reviewed_summaries_are_chinese(name):
    """摘要语言门禁不能代替完整契约审查."""
    path = Path(nav_bringup.__file__).parent / 'recorder' / name
    missing = []
    for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node)
            if doc and re.search(r'[\u4e00-\u9fff]', doc.splitlines()[0]) is None:
                missing.append(getattr(node, 'name', '<module>'))
    assert not missing, missing


def test_same_size_replaced_log_is_not_detected(tmp_path):
    """只按大小比较会跳过同路径等长替换，不声称支持完整轮转跟踪."""
    path = tmp_path / 'node.log'
    path.write_bytes(b'new!')
    linked = linked_process.LinkedProcess(17, (), None, (), {}, {str(path): 4})
    spec = linked_process.LinkSpec('loc', 'fixture', log_globs=(str(path),))
    destination = tmp_path / 'copy'
    assert linked_process.collect_logs(spec, linked, destination, since=0) == []
    assert not destination.exists()
    assert path.read_bytes() == b'new!'


def test_copy_failure_can_leave_unreported_partial_target(tmp_path, monkeypatch):
    """写入失败跳过报告但不回滚已写字节，源文件保持不变."""
    path = tmp_path / 'node.log'
    path.write_bytes(b'original')
    spec = linked_process.LinkSpec('loc', 'fixture', log_globs=(str(path),))

    def fail_copy(source, target, start, end):
        target.write_bytes(b'partial')
        raise OSError('fixture failure')

    monkeypatch.setattr(linked_process, '_copy_range', fail_copy)
    destination = tmp_path / 'copy'
    assert linked_process.collect_logs(spec, None, destination, since=0) == []
    assert (destination / 'node.log').read_bytes() == b'partial'
    assert path.read_bytes() == b'original'


def test_config_flags_preserve_empty_and_flag_like_values():
    """提取器不做命令行语义校验或去重."""
    command = ['fixture', '--config=', '--config', '--verbose', '--config=a']
    assert linked_process.config_paths(command, ['--config']) == ['', '--verbose', 'a']


def test_root_end_resets_attempts_not_throttle_history():
    """根结束后短间隔触发仍被抑制；恰好达到间隔时次数从一开始."""
    detector = bt_triggers.TriggerDetector(min_interval_sec=0.5)
    change = bt_triggers.StatusChange
    first = detector.process([change('Spin', 'IDLE', 'RUNNING', 10.0)])
    detector.process([change('NavigateRecovery', 'RUNNING', 'SUCCESS', 10.1)])
    assert detector.process([change('Spin', 'IDLE', 'RUNNING', 10.25)]) == []
    next_run = detector.process([change('Spin', 'IDLE', 'RUNNING', 10.5)])
    assert [item.attempt for item in first + next_run] == [1, 1]
    assert detector.suppressed == 1


def test_tree_clock_regression_is_suppressed_without_reset():
    """时间倒退按既有负差值限流，不自动重置探测器历史."""
    detector = bt_triggers.TriggerDetector(min_interval_sec=0.5)
    change = bt_triggers.StatusChange
    detector.process([change('Spin', 'IDLE', 'RUNNING', 10.0)])
    assert detector.process([change('Spin', 'IDLE', 'RUNNING', 1.0)]) == []
    resumed = detector.process([change('Spin', 'IDLE', 'RUNNING', 10.5)])
    assert resumed[0].attempt == 2
    assert detector.suppressed == 1


def test_event_data_omits_empty_task_id():
    """空任务号不输出，观测时间只取传入的行为树记录."""
    rule = bt_triggers.DEFAULT_RULES[0]
    trigger = bt_triggers.Trigger(
        rule, bt_triggers.StatusChange(rule.node_name, 'IDLE', 'SUCCESS', 1.23456789), 1)
    data = trigger.event_data(task_id='')
    assert 'task_id' not in data and 'pose' not in data
    assert data['bt_stamp'] == 1.234568
