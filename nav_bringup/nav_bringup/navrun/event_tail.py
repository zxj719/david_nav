"""在正常清理之后，持有一个有界的、与运动无关的诊断子进程.

它自己的生命周期事件只记在本地、来源标为 event_flush，不再向外发布——
否则每个发布者都需要另一个发布者来汇报自己的终止，形成无穷链条。
"""

import math
from pathlib import Path
import sys
import time
from typing import Mapping, Sequence, Tuple

from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import EventLog


def command(python: str = sys.executable) -> Tuple[str, ...]:
    """拼出这个私有辅助命令，既不导入 ROS 也不启动任何进程.

    Args:
        python: 父进程所选 ROS 环境里的绝对路径解释器。

    Returns:
        精确的参数向量；运行身份通过环境变量传递，不出现在命令行上。

    Raises:
        ValueError: 解释器不是绝对路径，或含 NUL 字符。
    """
    if not Path(python).is_absolute() or '\0' in python:
        raise ValueError('event flush interpreter must be an absolute path without NUL')
    return (python, '-m', 'nav_bringup.recorder.event_flush')


def flush_tail(
    argv: Sequence[str], run_dir: Path, environ: Mapping[str, str], events: EventLog,
    *, timeout_sec: float = 5.0,
) -> None:
    """尝试把最后一批事件发出去，且不改变已记录的导航状态.

    只能在包括 monitor 在内的全部原有子进程都已退干净、run_stopped 也已追加之后调用。
    调用期间租约仍由调用方持有。超出期限会中断这个私有辅助进程；清理会一直重试，
    直到可以安全释放归属为止。清理期间收到的中断要等排干之后才重新抛出——
    也就是说本函数宁可多等，也不留下无人回收的子进程。

    Args:
        argv: 显式的可选辅助命令向量；为空表示不做这一步。
        run_dir: 已存在且自有的运行目录，内含 logs 子目录。
        environ: 子进程环境，复制一份并补上运行身份。
        events: 父进程事件日志；辅助进程的结果用只记本地的来源写入。
        timeout_sec: 有限且为正的执行预算，不含清理重试的时间。

    Raises:
        ValueError: 显式给出的超时非法。
    """
    if not argv:
        return
    if (type(timeout_sec) not in (int, float) or not math.isfinite(timeout_sec)
            or timeout_sec <= 0):
        raise ValueError('event tail timeout must be finite and positive')
    child = None
    interrupted = None
    code = None
    failed = False
    cleanup_warned = False

    def record(name, **data):
        try:
            events.append('event_flush', name, 'info' if name == 'finished' else 'warn', data)
        except Exception:
            pass  # File diagnostics must never prevent owned group cleanup.

    try:
        child_env = dict(environ, NAVRUN_RUN_DIR=str(run_dir), NAVRUN_RUN_ID=run_dir.name)
        child = OwnedProcess(argv, child_env, run_dir / 'logs/event_flush.log')
        deadline = time.monotonic() + timeout_sec
        while child.poll() is None and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        code = child.poll()
        failed = code != 0
    except Exception as error:
        failed = True
        record('execution_failed', category=type(error).__name__)
    except BaseException as error:
        interrupted = error
        failed = True
    finally:
        while child is not None:
            try:
                child.close(1.0)
                break
            except Exception as error:
                failed = True
                if not cleanup_warned:
                    record('cleanup_pending', category=type(error).__name__)
                    cleanup_warned = True
            except BaseException as error:
                interrupted = interrupted or error
                failed = True
            try:
                time.sleep(0.1)
            except BaseException as error:
                interrupted = interrupted or error
        record('failed' if failed else 'finished', exit_code=code)
    if interrupted is not None:
        raise interrupted
