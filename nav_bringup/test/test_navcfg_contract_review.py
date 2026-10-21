"""纯配置边界：不连 ROS、不启动子进程、不写业务文件."""

import pytest

from nav_bringup.navcfg import cli, validation


def test_parser_construction_runs_no_subcommand(capsys):
    """只构建解析器不执行任何子命令，也不打印任何内容."""
    parser = cli.build_parser()
    assert parser.parse_args(['validate', '--robot', 'david', '--mode', 'tour']).command \
        == 'validate'
    assert capsys.readouterr().out == ''


@pytest.mark.parametrize('argv', [[], ['unknown_subcommand'], ['render']])
def test_malformed_invocations_use_the_documented_status(argv, capsys):
    """输入格式错误一律返回约定的 2，不返回校验失败用的 1."""
    try:
        status = cli.main(argv)
    except SystemExit as exit_error:  # argparse 对未知子命令直接退出。
        status = exit_error.code
    capsys.readouterr()
    assert status == 2


def test_fixed_joint_offset_ignores_movable_joints():
    """偏移只沿固定关节求解；可动关节不参与，因此结果与关节角无关."""
    urdf = (
        '<robot name="fixture">'
        '<joint name="a" type="fixed"><parent link="base_link"/>'
        '<child link="middle"/><origin xyz="0.2 0 0" rpy="0 0 0"/></joint>'
        '<joint name="b" type="fixed"><parent link="middle"/>'
        '<child link="base_footprint"/><origin xyz="0.1 0 -0.4" rpy="0 0 0"/></joint>'
        '</robot>')
    offset = validation.frame_offset(urdf, 'base_link', 'base_footprint')
    assert offset.xyz == pytest.approx((0.3, 0.0, -0.4))
    movable = urdf.replace('name="b" type="fixed"', 'name="b" type="revolute"')
    with pytest.raises(ValueError):
        validation.frame_offset(movable, 'base_link', 'base_footprint')


def test_unconnected_frames_are_an_error_not_a_zero_offset():
    """两个帧之间没有通路时报错，绝不返回零偏移冒充"已对齐"."""
    urdf = ('<robot name="fixture">'
            '<joint name="a" type="fixed"><parent link="base_link"/>'
            '<child link="middle"/><origin xyz="0.2 0 0" rpy="0 0 0"/></joint>'
            '</robot>')
    with pytest.raises(ValueError):
        validation.frame_offset(urdf, 'base_link', 'base_footprint')
