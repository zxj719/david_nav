"""Execute the opt-in, read-only live check while preserving caller shutdown ownership."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from typing import Any

from nav_bringup.navcfg.live_check import assess, Finding, wait_localization
from nav_bringup.navcfg.live_inputs import load_live_spec
from nav_bringup.navcfg.interface_check import check_interface
from nav_bringup.recorder.events import EventLog


def run(args: argparse.Namespace) -> int:
    """Check a prepared run; never start localization, actions, motion or a CLI daemon.

    Args:
        args: Explicit live selection, run_dir, strict flag, wait/sample/query bounds,
            and optional events path supplied by the owning supervisor.

    Returns:
        Zero after checks (warnings/errors printed), one for strict check errors,
        two for invalid inputs/operational failure, three while localization remains
        pending after an explicit diagnostic bound, or 128+signal on interruption.
        Wait-only mode returns zero for TF readiness without claiming full checks.

    Raises:
        ImportError: Required ROS runtime dependencies are unavailable.
    """
    if not args.live:
        print('navcfg check: select --live explicitly', file=sys.stderr)
        return 2
    stopped = threading.Event()
    received = []
    previous = {}

    def stop(signum: int, _frame: Any) -> None:
        received.append(signum)
        stopped.set()

    try:
        for value, minimum in ((args.wait_timeout, 0.0), (args.sample_sec, 0.0),
                               (args.parameter_timeout, 0.0)):
            if not math.isfinite(value) or value < minimum:
                raise ValueError('check durations must be finite and nonnegative')
        if args.sample_sec == 0.0 or args.parameter_timeout == 0.0:
            raise ValueError('sample and parameter query windows must be positive')
        spec, initial = load_live_spec(args.run_dir)
        events = EventLog(args.events) if args.events is not None else None

        def emit(finding: Finding) -> None:
            print(f'{finding.severity.upper()} [{finding.code}] {finding.message}', flush=True)
            if events is not None:
                events.append('navcfg', finding.code, finding.severity,
                              {'message': finding.message})

        # Import the ROS observer only after pure input validation succeeds.
        from nav_bringup.navcfg.live_ros import RosProbe

        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, stop)

        def _check_once() -> int:
            """Run one disposable observation; three means localization became pending."""
            with RosProbe(spec) as probe:
                ready = wait_localization(probe.localized, probe.spin, time.monotonic,
                                          stopped.is_set, emit, args.wait_timeout)
                if not ready:
                    return 128 + received[0] if received else 3
                if args.wait_localization_only:
                    return 0
                deadline = time.monotonic() + args.sample_sec
                while not stopped.is_set() and time.monotonic() < deadline:
                    probe.spin(max(0.0, min(0.1, deadline - time.monotonic())))
                if stopped.is_set():
                    return 128 + received[0]
                snapshot = probe.snapshot(stopped.is_set, args.parameter_timeout)
                if stopped.is_set():
                    return 128 + received[0]
            findings = initial + assess(spec, snapshot)
            if not snapshot.localized:
                for finding in findings:
                    emit(finding)
                return 3
            interface = check_interface(args.run_dir, args.sample_sec, stopped.is_set)
            if stopped.is_set() or interface is None:
                return 128 + received[0] if received else 130
            findings += interface.findings
            if events is not None:
                events.append('navcfg', 'interface_snapshot', 'info',
                              {'status': interface.status, 'evidence': interface.evidence})
            for finding in findings:
                emit(finding)
            if interface.status == 3:
                if not any(item.code == 'localization_waiting' for item in findings):
                    emit(Finding('info', 'localization_waiting', '等待在 RViz 手动给初始位姿'))
                return 3
            if interface.status == 0:
                emit(Finding('info', 'interface_contracts_ready',
                             'localization/chassis/perception contracts satisfied '
                             'in sample window'))
            errors = sum(item.severity == 'error' for item in findings)
            warnings = sum(item.severity == 'warn' for item in findings)
            print(f'navcfg check: {errors} error(s), {warnings} warning(s)', flush=True)
            return 1 if args.strict and errors else 0

        while True:
            result = _check_once()
            if result != 3 or args.wait_timeout > 0.0:
                return result
            emit(Finding('info', 'localization_retry',
                         'localization pending; waiting before a fresh full check'))
            if stopped.wait(0.25):
                return 128 + received[0] if received else 130
    except KeyboardInterrupt:
        return 130
    except (ValueError, LookupError, OSError, RuntimeError) as error:
        print(f'navcfg check: {error}', file=sys.stderr)
        return 128 + received[0] if received else 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
