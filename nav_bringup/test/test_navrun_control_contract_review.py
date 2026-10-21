"""纯文件/纯套接字边界，不启动 ROS、不发信号、不动实际运行记录."""

import pytest

from nav_bringup.navrun.control_server import RunControl
from nav_bringup.navrun.delegated_lock import DelegatedLock
from nav_bringup.navrun.locking import RunLock


def test_delegated_close_before_entry_is_a_noop_and_fileno_refuses(tmp_path):
    """未进入上下文时既没有副本可关，也不允许把原描述符当作本对象的能力."""
    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        borrowed = DelegatedLock(lease.fileno(), path)
        borrowed.close()  # 幂等：没有副本时什么都不做。
        with pytest.raises(RuntimeError):
            borrowed.fileno()
        with borrowed:
            duplicate = borrowed.fileno()
            assert duplicate != lease.fileno()
        with pytest.raises(RuntimeError):
            borrowed.fileno()  # 关闭之后能力同样消失。
        assert lease.fileno() >= 0  # 监督方的描述符全程未被关闭或解锁。


def test_delegated_second_entry_is_refused_without_touching_the_first(tmp_path):
    """重复进入被拒绝，且不影响已经校验过的副本."""
    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        with DelegatedLock(lease.fileno(), path) as borrowed:
            duplicate = borrowed.fileno()
            with pytest.raises(RuntimeError):
                borrowed.__enter__()
            assert borrowed.fileno() == duplicate
            borrowed.verify()


def test_control_poll_before_open_does_nothing(tmp_path):
    """端点未打开时 poll 直接返回，不报错也不写租约注记."""
    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        stops = []
        server = RunControl(lease, lambda: stops.append(True))
        before = path.read_bytes()
        server.poll()
        assert not stops and path.read_bytes() == before


def test_control_identify_before_open_annotates_without_an_endpoint(tmp_path):
    """记录身份可以在打开端点前公布；公布本身不创建端点、不请求停止."""
    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        stops = []
        server = RunControl(lease, lambda: stops.append(True))
        server.identify('fixture_run', 'precise')
        annotation = path.read_text(encoding='utf-8').splitlines()[0]
        assert 'fixture_run' in annotation and 'precise' in annotation
        assert not stops
        server.poll()  # 仍然没有端点，仍然是空操作。
        assert not stops


def test_control_second_entry_is_refused(tmp_path):
    """服务端已打开时重复进入被拒绝，原端点保持可用."""
    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        with RunControl(lease, lambda: None) as server:
            with pytest.raises(RuntimeError):
                server.__enter__()
            server.poll()
