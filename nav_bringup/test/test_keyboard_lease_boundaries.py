"""租约元数据与键盘通道说明所声明的边界（任务 P7.4）."""

import json
import os
import socket

import pytest

from nav_bringup.navrun.keyboard_channel import KeyboardChannel
from nav_bringup.navrun.keyboard_session import KeyboardSession
from nav_bringup.navrun.locking import RunLock


def test_annotate_leaves_exactly_one_record(tmp_path):
    """较短的新元数据必须截断旧内容，不留上一条的残尾."""
    path = tmp_path / 'lease'

    with RunLock(path) as lease:
        lease.annotate({'endpoint': 'x' * 200})
        lease.annotate({'endpoint': 'y'})
        content = path.read_bytes()

    assert content.count(b'\n') == 1
    assert json.loads(content.decode('utf-8'))['control'] == {'endpoint': 'y'}


@pytest.mark.parametrize('control', [{'blob': 'x' * 5000}, {'rate': float('inf')}])
def test_invalid_metadata_keeps_the_previous_record(tmp_path, control):
    """超限或非有限数值的元数据被拒绝，且不破坏已经写好的记录."""
    path = tmp_path / 'lease'

    with RunLock(path) as lease:
        lease.annotate({'endpoint': 'kept'})
        before = path.read_bytes()

        with pytest.raises(ValueError):
            lease.annotate(control)

        assert path.read_bytes() == before


def test_closing_the_duplicate_does_not_disconnect_the_owner():
    """关闭副本只关自己那一份：继承来的原描述符还开着时，持有方读不到 EOF."""
    parent, child = socket.socketpair()
    try:
        with KeyboardChannel(child.fileno()) as channel:
            channel.ready()
            assert parent.recv(1) == b'R'

        parent.setblocking(False)
        with pytest.raises(BlockingIOError):
            parent.recv(1)

        child.close()
        assert parent.recv(1) == b''
    finally:
        parent.close()
        child.close()


def test_session_ready_rejects_unexpected_child_data(tmp_path):
    """父进程侧同样 fail closed：子进程发来非就绪字节即报错，不当成还没就绪."""
    master, slave = os.openpty()
    try:
        with RunLock(tmp_path / 'lease') as lease:
            with KeyboardSession(lease, os.ttyname(slave)) as session:
                argv, _ = session.command(('managed',))
                control = int(argv[argv.index('--control-fd') + 1])
                with socket.socket(fileno=os.dup(control)) as child:
                    session.child_started()
                    child.sendall(b'X')

                    with pytest.raises(RuntimeError, match='invalid'):
                        session.ready()
    finally:
        os.close(master)
        os.close(slave)
