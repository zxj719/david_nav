"""进程/记录边界：只用私有临时目录与无害命令，不碰导航进程."""

import os
import time

import pytest

from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.navrun.startup_observer import StartupObserver


def _wait_exit(child, budget_sec=5.0):
    """等自有进程组的组长退出，返回其状态；超时判失败而不是继续等."""
    deadline = time.monotonic() + budget_sec
    while time.monotonic() < deadline:
        status = child.poll()
        if status is not None:
            return status
        time.sleep(0.02)
    pytest.fail('leader did not exit within the test budget')


@pytest.mark.parametrize('argv', [(), ('/bin/true', 5), ('/bin/true', 'a\0b')])
def test_invalid_commands_never_create_a_child_or_a_log(tmp_path, argv):
    """参数向量为空或含非字符串/NUL 时拒绝，且不创建子进程与日志文件."""
    log = tmp_path / 'child.log'
    with pytest.raises(ValueError):
        OwnedProcess(argv, {}, log)
    assert not log.exists()


def test_completed_close_is_idempotent_and_keeps_its_status(tmp_path):
    """组长自行退出后 close 取到其状态；重复 close 不再发信号，状态不变."""
    child = OwnedProcess(('/bin/sh', '-c', 'exit 7'), dict(os.environ), tmp_path / 'child.log')
    assert _wait_exit(child) == 7  # poll 观察到退出但不回收，身份仍被占住。
    first = child.close(timeout_sec=5.0)
    second = child.close(timeout_sec=5.0)
    assert first == 7 and second == 7


def test_child_runs_in_its_own_session_without_a_shell(tmp_path):
    """子进程独立成会话；参数逐项传递，不经 shell 解释."""
    log = tmp_path / 'child.log'
    child = OwnedProcess(('/bin/sh', '-c', 'ps -o sid= -p $$; echo "*"'),
                         dict(os.environ), log)
    _wait_exit(child)
    child.close(timeout_sec=5.0)
    lines = log.read_text(encoding='utf-8').split()
    assert int(lines[0]) != os.getsid(0)  # 不在测试进程的会话里。
    assert lines[1] == '*'  # 星号原样传给子进程，没有被外层展开。


def test_observer_construction_opens_no_file(tmp_path):
    """只描述记录器不启动时，不打开控制台文件、不建线程."""
    class _Run:
        """仅提供 path 的假运行目录."""

        path = tmp_path

    observer = StartupObserver(_Run(), None, None)
    assert observer.errors == []
    assert list(tmp_path.iterdir()) == []
