"""原 monitor 退干净之后，有界地把最后一段 NavEvent 发出去.

这是个短命的辅助进程，不持有任何运动接口。即使 ROS 初始化或发布卡住，
也必须由父进程监督并回收它的私有进程组——它自己不会脱管求生。
"""

import argparse
import math
import os
from pathlib import Path
import stat
import sys
import time
from typing import Callable, Optional, Sequence

from nav_bringup.recorder.run_event_stream import RunEventStream


def _notice(message):
    print(message, file=sys.stderr, flush=True)


def publish_tail(
    path: Path, *, timeout_sec: float = 3.0, discovery_sec: float = 0.5,
    warn: Callable[[str], None] = _notice,
) -> bool:
    """用一个临时诊断节点，尝试把剩下的时间线发完.

    初始化 ROS 之前先取同一把独占租约，保证不会和仍在运行的 monitor 重叠发布；
    身份与游标沿用持久化的那一份，不重新生成。有界的发现窗口与可靠写者的确认等待
    能提高送达率，但既不能证明订阅方处理过，也不能证明对方已持久化。
    对阻塞在原生 SDK 调用上的情况，由父进程提供独立的兜底期限。

    Args:
        path: 已存在运行的 events.jsonl；绝不重写或删除它。
        timeout_sec: 有限且为正的总协作预算，单位秒。
        discovery_sec: 初始发现窗口，取值在零与总预算之间。
        warn: 只收类别的诊断输出端，绝不收到事件内容。

    Returns:
        快照处理过程无告警且写者确认等待成功时返回 True。
        这只是发布侧的结果，不是任何导航结论。

    Raises:
        ValueError: 时间预算非法；该检查发生在取得 ROS 或文件归属之前。
    """
    if (type(timeout_sec) not in (int, float) or not math.isfinite(timeout_sec)
            or timeout_sec <= 0 or type(discovery_sec) not in (int, float)
            or not math.isfinite(discovery_sec) or not 0 <= discovery_sec <= timeout_sec):
        raise ValueError('event flush budgets must be finite, bounded and nonnegative')
    stream = node = context = ros = None
    failed = False
    succeeded = False

    def notice(category):
        nonlocal failed
        failed = True
        try:
            warn('final event publication: ' + category)
        except Exception:
            pass

    deadline = time.monotonic() + timeout_sec
    try:
        info = Path(path).lstat()
        if not stat.S_ISREG(info.st_mode):
            raise ValueError('event timeline must be a regular file')
        limit = info.st_size
        stream = RunEventStream(path, lambda values: publisher.publish(NavEvent(**values)), notice)
        from david_nav_msgs.msg import NavEvent
        import rclpy as ros
        from rclpy.context import Context
        from rclpy.duration import Duration

        context = Context()
        ros.init(args=[], context=context)
        node = ros.create_node('navrun_event_flush', context=context)
        publisher = node.create_publisher(NavEvent, '/nav/events', 10)
        discovery_end = min(deadline, time.monotonic() + discovery_sec)
        while time.monotonic() < discovery_end:
            remaining = max(0.0, discovery_end - time.monotonic())
            # Discovery is handled by DDS, not an application callback. Avoid
            # spin_once's global executor, which belongs to the default context.
            time.sleep(min(0.05, remaining))
        while time.monotonic() < deadline:
            before = stream.offset
            stream.pump()
            if stream.closed:
                break
            if stream.offset >= limit:
                remaining = max(0.0, deadline - time.monotonic())
                if not publisher.wait_for_all_acked(Duration(seconds=remaining)):
                    notice('acknowledgement timeout')
                succeeded = not failed
                break
            if stream.offset == before:
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        else:
            notice('timeline timeout')
    except Exception as error:
        notice(type(error).__name__)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception as error:
            notice(type(error).__name__)
        finally:
            try:
                if context is not None:
                    ros.try_shutdown(context=context)
            except Exception as error:
                notice(type(error).__name__)
            finally:
                if stream is not None:
                    stream.close()
    return succeeded and not failed


def main(argv: Optional[Sequence[str]] = None) -> int:
    """按父进程显式给出的运行路径，执行这条私有的末尾发布命令.

    Args:
        argv: 命令参数；为 None 时读取本进程参数。

    Returns:
        发布尝试成功返回 0，诊断失败返回 1；返回 0 只代表发布侧没有报错。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', default=os.environ.get('NAVRUN_RUN_DIR'), type=Path)
    args = parser.parse_args(argv)
    if (args.run_dir is None or not args.run_dir.is_absolute()
            or args.run_dir == Path('/') or '..' in args.run_dir.parts):
        parser.error('an explicit absolute run directory is required')
    return 0 if publish_tail(args.run_dir / 'events.jsonl') else 1


if __name__ == '__main__':
    raise SystemExit(main())
