"""纯对象边界：不启动进程、不发信号、不写运行记录."""

import subprocess
import sys

import pytest

from nav_bringup.navrun import recording
from nav_bringup.recorder import sessions


def test_disabled_recording_plans_nothing_and_says_why(tmp_path):
    """关闭录制时不拼命令、不建目录，并给出原因，而不是静默不录."""
    plan = recording.plan_recording({}, 'off', tmp_path, 'ros2')
    assert not plan.enabled and plan.argv == () and plan.reason
    assert list(tmp_path.iterdir()) == []


def test_light_recording_without_topics_is_disabled_not_widened(tmp_path):
    """轻量策略的话题列表为空即显式关闭，绝不回退成录所有话题."""
    profile = {'record': {'light_topics': []}}
    plan = recording.plan_recording(profile, 'light', tmp_path, 'ros2')
    assert not plan.enabled and plan.topics == () and plan.argv == ()


def test_already_exited_session_is_not_signalled():
    """已经退出的会话不再被发信号，直接返回它自己的退出状态."""
    process = subprocess.Popen([sys.executable, '-c', 'raise SystemExit(3)'],
                               start_new_session=True)
    assert process.wait(timeout=10) == 3
    assert sessions.stop_session(process, 1.0) == 3


@pytest.mark.parametrize('mode', ['sometimes', '', 'Light'])
def test_unknown_recording_modes_are_refused(tmp_path, mode):
    """录制策略只认 light/full/off，别的一律拒绝而不是猜测."""
    with pytest.raises(ValueError):
        recording.plan_recording({}, mode, tmp_path, 'ros2')
