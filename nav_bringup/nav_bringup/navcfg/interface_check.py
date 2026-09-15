"""Call the independently packaged interface observer without importing its runtime."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
from typing import Any, Callable, Mapping, Optional, Tuple

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.navcfg.live_check import Finding


@dataclass(frozen=True)
class InterfaceResult:
    """Validated version-one observer response.

    Attributes:
        status: Zero for satisfied contracts, one for errors, three for pending localization.
        findings: Structured contract findings, unchanged from the owning module.
        evidence: Window observations, or None while waiting without a sample.
    """

    status: int
    findings: Tuple[Finding, ...]
    evidence: Optional[Mapping[str, Any]]


def _evidence_valid(value):
    if not isinstance(value, dict) or set(value) != {
            'transforms', 'publishers', 'subscribers', 'samples', 'types', 'cloud_frame'}:
        return False

    def strings(items):
        return isinstance(items, list) and all(isinstance(i, str) for i in items)

    return (isinstance(value['transforms'], list)
            and all(strings(pair) and len(pair) == 2 for pair in value['transforms'])
            and strings(value['samples'])
            and isinstance(value['types'], dict)
            and all(isinstance(k, str) and strings(v) for k, v in value['types'].items())
            and (value['cloud_frame'] is None or isinstance(value['cloud_frame'], str))
            and all(isinstance(value[key], dict) and all(
                isinstance(topic, str) and type(count) is int and count >= 0
                for topic, count in value[key].items()) for key in ('publishers', 'subscribers')))


def parse_report(output: str, exit_code: int) -> InterfaceResult:
    """Check a single final JSON report and its agreement with process exit status.

    Args:
        output: Bounded combined output; non-JSON ROS log lines may precede the report.
        exit_code: Observed subprocess status after cleanup.

    Returns:
        Validated findings and raw window evidence for the navigation check.

    Raises:
        ValueError: Output is malformed, unsupported, missing or inconsistent.
    """
    documents = [json.loads(line) for line in output.splitlines() if line.startswith('{')]
    if type(exit_code) is not int or len(documents) != 1 or not isinstance(documents[0], dict):
        raise ValueError('interface checker must emit exactly one JSON result')
    data = documents[0]
    if (type(data.get('schema_version')) is not int or data['schema_version'] != 1
            or not isinstance(data.get('findings'), list)):
        raise ValueError('unsupported interface checker report')
    findings = []
    for item in data['findings']:
        if (not isinstance(item, dict) or set(item) != {'severity', 'code', 'message'}
                or item['severity'] not in ('info', 'warn', 'error')
                or not isinstance(item['code'], str) or not item['code']
                or not isinstance(item['message'], str)):
            raise ValueError('malformed interface finding')
        findings.append(Finding(**item))
    evidence = data.get('evidence')
    if evidence is not None and not _evidence_valid(evidence):
        raise ValueError('malformed interface evidence')
    errors = any(item.severity == 'error' for item in findings)
    if not ((exit_code in (0, 1) and data.get('status') == 'ready'
             and evidence is not None and bool(exit_code) == errors)
            or (exit_code == 3 and data.get('status') == 'waiting')):
        raise ValueError(f'interface checker failed or report disagrees with exit {exit_code}')
    return InterfaceResult(exit_code, tuple(findings), evidence)


def check_interface(
    run_dir: Path, sample_sec: float, stopped: Callable[[], bool],
) -> Optional[InterfaceResult]:
    """Own a bounded, read-only interface-check child and release it before returning.

    The child stays in the caller's process group, so an enclosing navrun owner
    can drain it even if this navcfg process is killed. It is never a detached
    helper. Temporary output is capped and removed only after child cleanup.

    Args:
        run_dir: Validated prepared run directory containing frozen robot/mode inputs.
        sample_sec: Positive data sampling window after the child's TF discovery.
        stopped: Caller-owned signal flag; cancellation returns None after cleanup.

    Returns:
        Validated result, or None when the caller requested interruption.

    Raises:
        ValueError: Invalid timing, missing executable or invalid observer output.
        LookupError: The separately installed observer package is missing.
        OSError: The child or its private output cannot be opened.
        RuntimeError: The observer exceeds its time/output budget.
    """
    if type(sample_sec) not in (int, float) or not math.isfinite(sample_sec) or sample_sec <= 0:
        raise ValueError('interface sample window must be finite and positive')
    if stopped():
        return None
    executable = resolve_executable('nav_perception_iface', 'nav_interface_check')
    command = [executable, '--profile', str(run_dir / 'params/inputs/robot.yaml'),
               '--mode', str(run_dir / 'params/inputs/mode.yaml'),
               '--sample-sec', str(sample_sec), '--wait-timeout', '5']
    with tempfile.TemporaryDirectory(prefix='navcfg-interface-') as directory:
        path = Path(directory) / 'observer.log'
        with path.open('xb') as output:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output,
                                       stderr=subprocess.STDOUT, close_fds=True,
                                       start_new_session=False)
        try:
            deadline = time.monotonic() + sample_sec + 10.0
            while process.poll() is None:
                if stopped():
                    return None
                if time.monotonic() >= deadline or path.stat().st_size > 1024 * 1024:
                    raise RuntimeError('interface checker exceeded its time/output budget')
                time.sleep(0.05)
            if stopped():
                return None
            if path.stat().st_size > 1024 * 1024:
                raise RuntimeError('interface checker exceeded its output budget')
            return parse_report(path.read_text(encoding='utf-8'), process.returncode)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
