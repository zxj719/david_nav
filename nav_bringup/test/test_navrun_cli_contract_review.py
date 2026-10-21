"""纯参数边界：不启动子进程、不创建运行目录、不碰 ROS."""

import pytest

from nav_bringup.navrun import teleop_options


def test_overrides_only_carry_what_was_given():
    """没写的字段不会被补默认值，冻结出来的就是操作者显式给的那几项."""
    assert teleop_options.TeleopOptions().values('teleop') == {}
    assert teleop_options.TeleopOptions(layout='wasd').values('teleop') == {'layout': 'wasd'}


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis'])
def test_overrides_outside_teleop_are_refused(mode):
    """键盘覆盖放错模式时在资源发现之前就拒绝."""
    with pytest.raises(ValueError):
        teleop_options.TeleopOptions(layout='wasd').values(mode)
