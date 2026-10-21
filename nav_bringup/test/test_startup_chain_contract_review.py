"""纯对象边界：不加锁、不创建运行目录、不启动任何进程."""

import signal

import pytest

from nav_bringup.navrun.recorded_startup import StartupInterrupted


def test_interruption_carries_its_signal_without_stopping_anything():
    """中断异常只带住第一个信号编号，本身不停止任何进程、不改全局处理器."""
    previous = signal.getsignal(signal.SIGINT)
    error = StartupInterrupted(signal.SIGINT)
    assert error.signum == signal.SIGINT
    assert signal.getsignal(signal.SIGINT) is previous
    with pytest.raises(StartupInterrupted) as caught:
        raise error
    assert caught.value.signum == signal.SIGINT
