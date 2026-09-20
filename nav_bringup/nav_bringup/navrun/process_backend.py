"""预检进程观察与身份安全发信号的显式 Linux 适配层.

导入本模块不会对任何进程做任何事。测试用假件替换 psutil 与 pidfd 操作，
普通测试套件从不真的发出信号。
"""

from __future__ import annotations

import os
import signal
from typing import Tuple

import psutil

from nav_bringup.navrun.preflight import PreflightError, ProcessInfo


def _read(process: psutil.Process) -> ProcessInfo:
    """在 psutil oneshot 中读取进程身份，只返回内存快照不输出命令行.

    读取 PID、父 PID、创建时间、argv 和真实 UID；不是原子身份锁，
    需要发信号的调用方仍须固定并复核实际进程。

    Args:
        process: 指定进程的 psutil 只读访问对象。

    Returns:
        用于前置检查与后续身份比对的 ProcessInfo。

    Raises:
        psutil.Error: 进程消失、权限不足或属性查询失败。
    """
    with process.oneshot():
        return ProcessInfo(process.pid, process.ppid(), process.create_time(),
                           tuple(process.cmdline()), process.uids().real)


def snapshot_processes() -> Tuple[ProcessInfo, ...]:
    """取一份完整的进程视图，不用 ROS、不起子进程、不提权.

    观察期间自行退出的进程会被略过；其它读取错误一律否决整份快照——
    读不到某个进程不等于它不存在。命令行只留在内存里，本模块不打印也不落盘。

    Returns:
        按 PID 排序的进程身份，含规划器所需的父子关系。

    Raises:
        PreflightError: 权限拒绝或其它错误导致无法取得完整视图。
    """
    processes = []
    try:
        for process in psutil.process_iter():
            try:
                processes.append(_read(process))
            except psutil.NoSuchProcess:
                continue
    except (psutil.Error, OSError) as error:
        raise PreflightError('cannot obtain complete process snapshot') from error
    return tuple(sorted(processes, key=lambda process: process.pid))


def send_if_same(expected: ProcessInfo, signum: int) -> bool:
    """用 pidfd 钉住进程，复核身份，再发出请求的信号.

    这里**故意**没有 os.kill 兜底：pidfd 在目标退出后仍指向原来那个进程，
    不会误伤后来复用同一个 PID 号的新进程，而 os.kill 会。
    调用方必须使用预检规划器给出的归属与保护判断，不得自行挑目标。

    Args:
        expected: 来自清理规划器、刚刚判定为可处理的进程身份。
        signum: 只允许 SIGTERM 或 SIGKILL，不接受任意信号。

    Returns:
        被钉住的身份确实收到信号返回 True；发送前它消失或身份改变返回 False。
        权限错误不算成功。

    Raises:
        ValueError: 信号不在白名单、PID 是特殊值，或身份属于其他用户。
        PreflightError: pidfd 不可用，或无法检查该进程。
        OSError: 操作系统拒绝打开 pidfd 或拒绝发信号。
    """
    if (signum not in (signal.SIGTERM, signal.SIGKILL) or expected.pid <= 1
            or expected.pid == os.getpid() or expected.uid != os.getuid()):
        raise ValueError('only eligible same-user non-self processes may receive TERM or KILL')
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise PreflightError('pidfd support is required; refusing PID-only signaling')
    try:
        descriptor = os.pidfd_open(expected.pid, 0)
    except ProcessLookupError:
        return False
    except NotImplementedError as error:
        raise PreflightError('pidfd is unavailable on this platform') from error
    try:
        try:
            current = _read(psutil.Process(expected.pid))
        except psutil.NoSuchProcess:
            return False
        except psutil.Error as error:
            raise PreflightError('cannot recheck process identity before signaling') from error
        if ((current.pid, current.started_at, current.uid, current.argv)
                != (expected.pid, expected.started_at, expected.uid, expected.argv)):
            return False
        try:
            signal.pidfd_send_signal(descriptor, signum, None, 0)
        except ProcessLookupError:
            return False
        return True
    finally:
        os.close(descriptor)
