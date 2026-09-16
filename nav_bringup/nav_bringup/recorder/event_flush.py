"""Bounded final NavEvent publication after the original monitor has been drained.

This short-lived helper owns no motion interfaces. Its parent must supervise and
reap its private process group even if ROS initialization or publication stalls.
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
    """Attempt the remaining final timeline through a temporary diagnostic node.

    Acquire the same exclusive lease before initializing ROS, so a live monitor
    cannot overlap this publisher. Reuse persistent IDs and cursor. A bounded
    discovery window and reliable-writer acknowledgement wait improve delivery,
    but neither proves subscriber application processing or durable recording.
    The parent provides an independent deadline for blocking native SDK calls.

    Args:
        path: Existing run's events.jsonl; never rewritten or removed.
        timeout_sec: Positive finite overall cooperative budget in seconds.
        discovery_sec: Initial discovery window, between zero and the total budget.
        warn: Category-only diagnostic sink; never receives event contents.

    Returns:
        True if the snapshot was processed without warnings and the writer's
        acknowledgement wait succeeded. This is not a navigation result.

    Raises:
        ValueError: A time budget is invalid, before any ROS or file ownership.
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
    """Run the private final-publisher command using the parent's explicit run path.

    Args:
        argv: Command arguments; None reads process arguments.

    Returns:
        Zero for a successful publication attempt, one for diagnostics failure.
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
