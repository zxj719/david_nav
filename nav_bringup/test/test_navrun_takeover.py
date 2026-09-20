"""接管只协作、不强夺：私有租约与真实协作端点，全程不发操作系统信号、不启动导航."""

import os
from pathlib import Path
import threading
import time

import pytest

from nav_bringup.navrun.control_server import RunControl
from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.up import take_over


@pytest.fixture(autouse=True)
def forbid_signals(monkeypatch):
    """接管路径一旦对任何进程发信号即判失败."""
    monkeypatch.setattr(os, 'kill', lambda *args: pytest.fail('takeover must never signal'))
    monkeypatch.setattr(os, 'killpg', lambda *args: pytest.fail('takeover must never signal'))


def test_free_lease_is_reported_inactive_without_touching_it(tmp_path):
    """没人持有时立即返回，且不创建租约文件."""
    path = tmp_path / 'lease'
    result = take_over(path, 5.0)
    assert result['state'] == 'inactive' and not result.get('cleanup_complete')
    assert not path.exists()


@pytest.mark.parametrize('budget', [0.0, -1.0, float('inf'), float('nan')])
def test_invalid_budget_is_refused_before_contacting_anyone(tmp_path, budget):
    """预算非法在联系持有者之前就拒绝."""
    with pytest.raises(ValueError):
        take_over(tmp_path / 'lease', budget)


def test_legacy_busy_lease_is_refused_and_keeps_its_lock(tmp_path):
    """持有者没有协作端点时拒绝接管，锁仍归原持有者，绝不夺取."""
    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        with pytest.raises(RuntimeError, match='not a cooperative navrun'):
            take_over(path, 1.0)
        assert lease.fileno() >= 0  # 原持有者的描述符全程未被动过。
        second = RunLock(path)
        with pytest.raises(Exception):
            second.acquire()  # 锁仍被占着，接管失败没有腾出位置。


def test_owner_that_does_not_finish_blocks_the_takeover(tmp_path):
    """持有者接受了请求但没清理完，预算耗尽即失败，不硬闯."""
    path = tmp_path / 'lease'
    stopped, finished = threading.Event(), threading.Event()
    with RunLock(path) as lease, RunControl(lease, stopped.set) as server:
        server.identify('fixture_run', 'precise')

        def poll():
            while not finished.wait(0.005):
                server.poll()

        thread = threading.Thread(target=poll)
        thread.start()
        try:
            with pytest.raises(RuntimeError, match='takeover incomplete'):
                take_over(path, 0.4)
            assert stopped.is_set()  # 请求确实送到了，只是持有者还没退完。
        finally:
            finished.set()
            thread.join(timeout=2)
            assert not thread.is_alive()


def test_cooperative_owner_releases_and_takeover_succeeds(tmp_path):
    """持有者收到请求后退出上下文：接管等到 cleanup_complete 才返回."""
    path = tmp_path / 'lease'
    stopped = threading.Event()
    failures = []

    def owner():
        """持有租约并轮询，直到收到停止请求后正常收尾释放."""
        try:
            with RunLock(path) as lease, RunControl(lease, stopped.set) as server:
                server.identify('fixture_run', 'precise')
                while not stopped.is_set():
                    server.poll()
                    time.sleep(0.005)
        except Exception as error:  # 线程内异常要带回主线程，避免静默通过。
            failures.append(error)

    thread = threading.Thread(target=owner)
    thread.start()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            # 等到注记里真的出现协作端点为止，只有文件存在还不够。
            if path.exists() and 'control' in path.read_text(encoding='utf-8'):
                break
            time.sleep(0.01)
        result = take_over(path, 5.0)
        assert result['cleanup_complete'] is True
        assert result['state'] == 'finished' and result['run_id'] == 'fixture_run'
    finally:
        stopped.set()
        thread.join(timeout=5)
        assert not thread.is_alive()
    assert not failures, failures
    RunLock(path).acquire()  # 位置确实腾出来了：接管方现在拿得到锁。


def test_start_only_takes_over_when_explicitly_asked(tmp_path, monkeypatch):
    """默认不接管：没有 --takeover 时启动路径根本不联系持有者."""
    from nav_bringup.navrun import up

    monkeypatch.setattr(up, 'take_over',
                        lambda *args: pytest.fail('takeover must be opt-in'))
    request = type('Args', (), {'takeover': False, 'repo': None, 'source_root': None})()
    with pytest.raises(Exception):
        up.start(request)  # 没有 takeover 就直接走既有路径（这里因缺参数而失败）。
    assert not (Path(tmp_path) / 'lease').exists()
