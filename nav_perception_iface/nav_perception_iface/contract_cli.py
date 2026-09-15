"""Independent read-only interface probe, suitable for navcfg subprocess integration."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Optional, Sequence

from nav_perception_iface.configuration import load_contract_profile
from nav_perception_iface.contracts import check_contracts


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Wait for manual localization, then report actual interface evidence as JSON.

    Args:
        argv: CLI arguments, or process arguments when omitted. --profile and
            optional --mode are explicit files; no provider is selected or started.

    Returns:
        Zero for satisfied contracts, one for contract errors, two for operational
        errors, three for bounded pending localization, or 128+signal on interruption.
        Stdout contains one versioned JSON report; waiting hints go to stderr.

    Raises:
        ImportError: The explicitly requested ROS runtime dependencies are unavailable.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--mode', type=Path)
    parser.add_argument('--sample-sec', type=float, default=2.0)
    parser.add_argument('--wait-timeout', type=float, default=0.0)
    parser.add_argument('--no-chassis', action='store_true',
                        help='explicit standalone diagnostic without a driver contract')
    args = parser.parse_args(argv)
    received = []
    previous = {}
    document = []

    def stop(signum, _frame):
        received.append(signum)

    def report(status, findings=(), snapshot=None):
        evidence = None if snapshot is None else {
            'transforms': sorted(snapshot.transforms), 'publishers': dict(snapshot.publishers),
            'subscribers': dict(snapshot.subscribers), 'samples': sorted(snapshot.samples),
            'types': {key: sorted(value) for key, value in snapshot.types.items()},
            'cloud_frame': snapshot.cloud_frame}
        # A close failure can replace this candidate before anything is emitted.
        document[:] = [{'schema_version': 1, 'status': status,
                        'findings': [asdict(item) for item in findings], 'evidence': evidence}]

    try:
        if (not math.isfinite(args.sample_sec) or args.sample_sec <= 0
                or not math.isfinite(args.wait_timeout) or args.wait_timeout < 0):
            raise ValueError('sample window must be positive; wait timeout must be nonnegative')
        profile = load_contract_profile(args.profile, args.mode)
        from nav_perception_iface.contract_probe import ContractProbe

        # Validate the contract before creating a context or changing signal handlers.
        probe = ContractProbe(profile, chassis_required=not args.no_chassis)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, stop)
        with probe:
            started = next_hint = time.monotonic()
            while not received and not probe.localized():
                now = time.monotonic()
                if args.wait_timeout and now - started >= args.wait_timeout:
                    report('waiting')
                    return 3
                if now >= next_hint:
                    print('等待在 RViz 手动给初始位姿', file=sys.stderr, flush=True)
                    next_hint = now + 5.0
                remaining = args.wait_timeout - (now - started) if args.wait_timeout else 0.1
                probe.spin(max(0.0, min(0.1, remaining)))
            if received:
                report('interrupted')
                return 128 + received[0]
            snapshot = probe.sample(args.sample_sec, lambda: bool(received))
            if received:
                report('interrupted')
                return 128 + received[0]
            findings = check_contracts(profile, snapshot, chassis_required=not args.no_chassis)
            if any(item.code == 'localization_waiting' for item in findings):
                report('waiting', findings, snapshot)
                return 3
            report('ready', findings, snapshot)
            return int(any(item.severity == 'error' for item in findings))
    except (ValueError, OSError, RuntimeError) as error:
        print(f'nav_interface_check: {error}', file=sys.stderr)
        report('error')
        return 128 + received[0] if received else 2
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        if document:
            print(json.dumps(document[0], ensure_ascii=False, allow_nan=False), flush=True)
