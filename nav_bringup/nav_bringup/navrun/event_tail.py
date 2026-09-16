"""Own a bounded, non-motion diagnostic child after normal run cleanup.

Its lifecycle observations are local-only with source event_flush, avoiding an
infinite chain of publishers needed to report their own final termination.
"""

import math
from pathlib import Path
import sys
import time
from typing import Mapping, Sequence, Tuple

from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import EventLog


def command(python: str = sys.executable) -> Tuple[str, ...]:
    """Build the private helper command without importing ROS or starting processes.

    Args:
        python: Absolute interpreter in the parent's selected ROS environment.

    Returns:
        Exact argument vector; run identity is passed through the environment.

    Raises:
        ValueError: Interpreter is not an absolute path or contains NUL.
    """
    if not Path(python).is_absolute() or '\0' in python:
        raise ValueError('event flush interpreter must be an absolute path without NUL')
    return (python, '-m', 'nav_bringup.recorder.event_flush')


def flush_tail(
    argv: Sequence[str], run_dir: Path, environ: Mapping[str, str], events: EventLog,
    *, timeout_sec: float = 5.0,
) -> None:
    """Attempt final event publication without changing the recorded navigation status.

    Call only after every original child, including the monitor, has drained and
    run_stopped has been appended. The caller retains its lease across this call.
    A deadline interrupts this private helper; cleanup is retried until ownership
    can be safely released. Cleanup interruption is re-raised only after draining.

    Args:
        argv: Explicit optional helper vector; empty disables the operation.
        run_dir: Existing owned run directory with a logs subdirectory.
        environ: Child environment, copied and supplemented with run identity.
        events: Parent event log; helper outcome uses a local-only source.
        timeout_sec: Positive finite execution budget, excluding cleanup retries.

    Raises:
        ValueError: The explicit timeout is invalid.
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
