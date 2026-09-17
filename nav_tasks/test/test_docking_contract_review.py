"""保护已审 Dock 策略说明并检查停稳阈值附近的纯样本，不替代实车验收."""

import ast
from pathlib import Path
import re

import pytest

from nav_tasks.skills.docking.settle import is_settled, SettleSpec


@pytest.mark.parametrize('path, qualified_name', [
    ('skills/__init__.py', '<module>'),
    ('skills/docking/__init__.py', '<module>'),
    ('skills/docking/ports.py', '<module>'),
    ('skills/docking/ports.py', 'DockActionResult'),
    ('skills/docking/ports.py', 'DockMotionPort'),
    ('skills/docking/ports.py', 'DockMotionPort.navigate'),
    ('skills/docking/ports.py', 'DockMotionPort.spin'),
    ('skills/docking/ports.py', 'DockLogger'),
    ('skills/docking/ports.py', 'DockLogger.info'),
    ('skills/docking/ports.py', 'DockLogger.warn'),
    ('skills/docking/ports.py', 'DockLogger.error'),
    ('skills/docking/approach.py', '<module>'),
    ('skills/docking/approach.py', 'check_approach_footprint'),
    ('skills/docking/settle.py', '<module>'),
    ('skills/docking/settle.py', 'SettleSpec'),
    ('skills/docking/settle.py', 'is_settled'),
    ('skills/docking/settle.py', 'wait_until_settled'),
    ('skills/docking/finish.py', '<module>'),
    ('skills/docking/finish.py', 'DockTolerance'),
    ('skills/docking/finish.py', 'DockOutcome'),
    ('skills/docking/finish.py', 'finish_docking'),
    ('skills/docking/workflow.py', '<module>'),
    ('skills/docking/workflow.py', 'DockConfig'),
    ('skills/docking/workflow.py', 'DockWorkflow'),
    ('skills/docking/workflow.py', 'DockWorkflow.__init__'),
    ('skills/docking/workflow.py', 'DockWorkflow.execute'),
    ('skills/docking/config.py', '<module>'),
    ('skills/docking/config.py', 'workflow_config'),
    ('skills/docking/parameters.py', '<module>'),
    ('skills/docking/parameters.py', 'from_parameters'),
])
def test_reviewed_docking_summary(path, qualified_name):
    """逐项保护已审摘要，策略等价仍由 AST 和行为测试验证.

    Args:
        path: 包内模块相对路径。
        qualified_name: 模块、类或方法的限定名。
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
    assert re.search('[\\u4e00-\\u9fff]', summary), qualified_name


@pytest.mark.parametrize('rate, expected', [
    (0.0499, True),
    (-0.0499, True),
    (0.0501, False),
    (-0.0501, False),
])
def test_settle_rate_near_requested_threshold(rate, expected):
    """检查正反向角速度在用户要求阈值两侧的结果，不调用动作或 ROS.

    Args:
        rate: 合成的恒定偏航角速度（弧度/秒）。
        expected: 完整窗口的预期判定。
    """
    spec = SettleSpec()
    assert spec.max_rate_rad_s == 0.05
    assert spec.window_sec == 0.4
    assert spec.sample_period_sec == 0.05
    samples = [(index * 0.05, rate * index * 0.05) for index in range(9)]
    assert is_settled(samples, spec) is expected
