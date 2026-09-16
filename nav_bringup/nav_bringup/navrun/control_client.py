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
    pid, uid, _ = struct.unpack('3i', sock.getsockopt(
        socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i')))
    return pid, uid


def _exchange(path: Path, operation: str, timeout: float) -> Dict[str, Any]:
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
