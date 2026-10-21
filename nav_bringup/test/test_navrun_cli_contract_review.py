"""纯参数边界：不启动子进程、不创建运行目录、不碰 ROS."""

import pytest

from nav_bringup.navrun import event_tail, teleop_options


def test_overrides_only_carry_what_was_given():
    """没写的字段不会被补默认值，冻结出来的就是操作者显式给的那几项."""
    assert teleop_options.TeleopOptions().values('teleop') == {}
    assert teleop_options.TeleopOptions(layout='wasd').values('teleop') == {'layout': 'wasd'}


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis'])
def test_overrides_outside_teleop_are_refused(mode):
    """键盘覆盖放错模式时在资源发现之前就拒绝."""
    with pytest.raises(ValueError):
        teleop_options.TeleopOptions(layout='wasd').values(mode)


@pytest.mark.parametrize('python', ['python3', '/usr/bin/pyt\0hon3'])
def test_helper_command_requires_an_absolute_clean_interpreter(python):
    """解释器必须是绝对路径且不含 NUL；拒绝发生在拼命令阶段."""
    with pytest.raises(ValueError):
        event_tail.command(python)


def test_helper_command_keeps_run_identity_out_of_the_argument_vector():
    """运行身份走环境变量，不出现在命令行上."""
    command = event_tail.command('/usr/bin/python3')
    assert command[0] == '/usr/bin/python3'
    assert not any('nav_runs' in part for part in command)
