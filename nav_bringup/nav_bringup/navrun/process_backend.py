"""Explicit Linux adapters for preflight process observation and identity-safe signals.

Importing this module does nothing to processes. Tests replace psutil and pidfd
operations with fakes; no real signal is sent by the ordinary test suite.
"""

from __future__ import annotations

import os
import signal
from typing import Tuple

import psutil

from nav_bringup.navrun.preflight import PreflightError, ProcessInfo


def _read(process: psutil.Process) -> ProcessInfo:
    with process.oneshot():
        return ProcessInfo(process.pid, process.ppid(), process.create_time(),
                           tuple(process.cmdline()), process.uids().real)


def snapshot_processes() -> Tuple[ProcessInfo, ...]:
    """Read a complete process view without ROS, subprocesses or privilege escalation.

    Processes that exit during observation are omitted. Other inspection errors
    reject the snapshot; an unreadable process is not evidence that it is absent.
    Command lines are returned in memory only, never printed or logged here.

    Returns:
        Process identities sorted by PID, including ancestry needed by the planner.

    Raises:
        PreflightError: Access denial or another error prevents a complete view.
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
    """Pin a process using pidfd, recheck identity, then send the requested signal.

    There is deliberately no os.kill fallback. A pidfd that outlives an exiting
    target cannot address the new process that later reuses the numeric PID.
    The caller must use the preflight planner's ownership/protection decisions.

    Args:
        expected: Freshly eligible process identity from the cleanup planner.
        signum: SIGTERM or SIGKILL, never an arbitrary signal.

    Returns:
        True if the pinned identity received a signal, False if it disappeared
        or changed before sending. Permission errors are not treated as success.

    Raises:
        ValueError: A disallowed signal, special PID or other user's identity.
        PreflightError: pidfd is unavailable or the process cannot be inspected.
        OSError: The operating system refuses opening or signaling the pidfd.
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
