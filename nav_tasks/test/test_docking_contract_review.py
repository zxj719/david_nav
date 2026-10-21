"""保护已审 Dock 策略说明并检查停稳阈值附近的纯样本，不替代实车验收."""

import pytest

from nav_tasks.skills.docking.settle import is_settled, SettleSpec


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
