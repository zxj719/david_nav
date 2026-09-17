"""Read-only lease inspection and cooperative shutdown, never PID-based signaling."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import socket
import stat
import struct
import time
from typing import Any, Dict, Tuple


def _snapshot(path: Path) -> Tuple[bool, Dict[str, Any], Tuple[int, int]]:
    """只读探测租约锁和有界元数据，并复核读取期间 inode 身份未变.

    Args:
        path: 当前用户的导航租约文件路径，不允许链接或组/其它用户写入。

    Returns:
        锁是否被占用、字典元数据、设备与 inode 三元组；不存在时为未持有，
        非法 JSON 返回空字典。短暂共享锁探测结束后释放，不删除租约。

    Raises:
        ValueError: 文件安全属性、尺寸或身份复核失败。
        OSError: 租约无法打开、读取或检查。
    """
    try:
        before = path.lstat()
        if (not stat.S_ISREG(before.st_mode) or before.st_uid != os.geteuid()
                or before.st_nlink != 1 or before.st_mode & 0o022):
            raise ValueError('unsafe navigation lease')
        descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK)
    except FileNotFoundError:
        return False, {}, (0, 0)
    try:
        info = os.fstat(descriptor)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid()
                or info.st_nlink != 1 or info.st_mode & 0o022):
            raise ValueError('unsafe navigation lease')
        held = True
        try:
            fcntl.flock(descriptor, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            pass
        else:
            held = False
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        payload = os.pread(descriptor, 4097, 0)
        if len(payload) > 4096:
            raise ValueError('oversized lease metadata')
        current = path.lstat()
        if (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino):
            raise ValueError('navigation lease changed during inspection')
        try:
            data = json.loads(payload)
        except (ValueError, RecursionError):
            data = {}
        return held, data if isinstance(data, dict) else {}, (info.st_dev, info.st_ino)
    finally:
        os.close(descriptor)


def _peer(sock: socket.socket) -> Tuple[int, int]:
    """从已连接 Unix socket 的内核凭据读取对端 PID 与 UID.

    忽略 GID；本函数只提供身份数据，授权判定仍由调用方完成。

    Args:
        sock: 已建立连接、支持 Linux SO_PEERCRED 的 socket。

    Returns:
        内核报告的对端 PID 和 UID 二元组。

    Raises:
        OSError: 凭据读取失败或平台/连接不支持该选项。
    """
    pid, uid, _ = struct.unpack('3i', sock.getsockopt(
        socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i')))
    return pid, uid


def _exchange(path: Path, operation: str, timeout: float) -> Dict[str, Any]:
    """通过租约绑定的 Unix 对端交换一条协作请求，发送前再次核对实例身份.

    Args:
        path: 已知导航租约路径。
        operation: 调用方选择的 status 或 down 操作。
        timeout: 总通信预算（单调时钟秒）。

    Returns:
        经实例/PID/状态核对的响应，并附租约 inode；无持有者时返回 inactive，
        不以不存在的进程推断清理已完成。

    Raises:
        ValueError: 租约协议、同用户对端、实例身份或响应内容不符。
        TimeoutError: 连接/通信期限耗尽。
        OSError: 租约读取或 socket 操作失败；不会改用信号搜杀进程。
    """
    held, data, identity = _snapshot(path)
    if not held:
        return {'state': 'inactive', 'cleanup_complete': False}
    control = data.get('control')
    if (type(data.get('pid')) is not int or data['pid'] <= 0
            or not isinstance(control, dict) or control.get('version') != 1
            or not isinstance(control.get('instance'), str)
            or re.fullmatch('[0-9a-f]{32}', control['instance']) is None
            or control.get('address') != f'navrun_{os.geteuid()}_{control["instance"]}'):
        raise ValueError('held lease has no compatible cooperative endpoint')
    deadline = time.monotonic() + timeout

    def remaining() -> float:
        duration = deadline - time.monotonic()
        if duration <= 0:
            raise TimeoutError('control response deadline expired')
        return duration

    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as sock:
        sock.settimeout(remaining())
        sock.connect('\0' + control['address'])
        if _peer(sock) != (data['pid'], os.geteuid()):
            raise ValueError('cooperative endpoint peer identity mismatch')
        still_held, current, current_identity = _snapshot(path)
        if not still_held or current_identity != identity or current != data:
            raise ValueError('lease incarnation changed before request')
        sock.settimeout(remaining())
        sock.sendall(json.dumps({'op': operation, 'instance': control['instance']}).encode())
        sock.settimeout(remaining())
        packet, _, flags, _ = sock.recvmsg(4096)
        if flags & socket.MSG_TRUNC:
            raise ValueError('oversized control response')
        result = json.loads(packet)
    if (not isinstance(result, dict) or result.get('instance') != control['instance']
            or result.get('pid') != data['pid']
            or result.get('state') not in ('active', 'stop_requested')
            or operation == 'down' and result['state'] != 'stop_requested'):
        raise ValueError('invalid cooperative response')
    result['_identity'] = identity
    return result


def _wait(path: Path, result: Dict[str, Any], timeout: float) -> Dict[str, Any]:
    """轮询同一租约实例的最终标记，不把释放锁或超时本身当清理成功.

    Args:
        path: 之前请求所绑定的租约路径。
        result: 已验证响应，包含实例及私有 inode 身份。
        timeout: 继续等待的单调时钟预算（秒），轮询间隔最多 0.05 秒。

    Returns:
        同实例释放且标记 finished 才返回 cleanup_complete=True；换代或无终态
        返回 unknown，预算耗尽保留原响应。

    Raises:
        ValueError: 租约安全复核失败。
        OSError: 租约读取失败。
    """
    deadline = time.monotonic() + timeout
    while True:
        held, data, identity = _snapshot(path)
        control = data.get('control', {})
        if (identity != result['_identity'] or not isinstance(control, dict)
                or control.get('instance') != result['instance']):
            return dict(result, state='unknown', reason='lease incarnation changed')
        if not held:
            if control.get('finished') is True:
                return dict(result, state='finished', cleanup_complete=True)
            return dict(result, state='unknown', reason='owner released without finalization')
        if time.monotonic() >= deadline:
            return result
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))


def run(args: argparse.Namespace) -> int:
    """Print current managed-owner status or request and await cooperative cleanup.

    Args:
        args: status/down, lock_path, positive exchange timeout and nonnegative wait bound.

    Returns:
        Zero for a verified response/completion or no managed lease; two for unknown
        identity/operational failure, three for accepted but incomplete cleanup.
        A client keyboard interruption returns 130 without forcing owner termination.
        Inactive never claims that legacy or manually launched processes are stopped.
    """
    interrupted = False
    try:
        if (not math.isfinite(args.timeout) or args.timeout <= 0
                or not math.isfinite(args.wait_timeout) or args.wait_timeout < 0):
            result = {'state': 'unknown', 'reason': 'invalid control timeout'}
        else:
            result = _exchange(args.lock_path.expanduser(), args.subcommand, args.timeout)
            if args.subcommand == 'down' and result['state'] == 'stop_requested':
                result = _wait(args.lock_path.expanduser(), result, args.wait_timeout)
    except (OSError, ValueError, RuntimeError) as error:
        result = {'state': 'unknown', 'reason': type(error).__name__}
    except KeyboardInterrupt:
        interrupted = True
        result = {'state': 'unknown', 'reason': 'client interrupted; owner not forced'}
    public = {key: value for key, value in result.items()
              if key in ('state', 'pid', 'run_id', 'mode', 'cleanup_complete', 'reason')}
    print(json.dumps(public, ensure_ascii=False, sort_keys=True))
    if interrupted:
        return 130
    if result['state'] == 'unknown':
        return 2
    if args.subcommand == 'down' and result['state'] == 'stop_requested':
        return 3
    return 0
