"""Dump the effective parameters of every node of a running ROS graph.

The run recorder keeps what the nodes actually use, after launch
substitutions, overrides and defaults, in ``params/effective/`` (refactor plan
§13.2). Dumps go through the ``ros2`` command line without the CLI daemon,
because a stale daemon silently returns an empty graph.

The rules come from the golden capture of PROGRESS P0.7
(``david_nav/test/golden/capture_nav2_param_golden.sh``):

* helper nodes that expose no parameter services make ``ros2 param dump``
  block forever, so they are skipped;
* discovery without the daemon is occasionally too slow, so every call has a
  timeout, dumps are retried, and a dump must produce non-empty output.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import fnmatch
import functools
from pathlib import Path
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

import yaml

from nav_bringup.recorder import manifest

# Humble ComponentManager explicitly disables its own parameter services. Only
# our exact helper name is excluded; every loaded component remains in the dump.
DEFAULT_SKIP_PATTERNS = (
    '*_rclcpp_node', '*/transform_listener_impl_*', '*/_ros2cli_*', '/nav2_container')
NODES_FILE_NAME = 'nodes.txt'
REPORT_FILE_NAME = 'dump_report.yaml'


@dataclass(frozen=True)
class DumpSettings:
    """How nodes are listed and dumped.

    Attributes:
        spin_time_sec: Discovery time passed to ``--spin-time``.
        timeout_sec: Seconds before one ``ros2`` call is abandoned.
        attempts: Dump attempts per node.
        retry_delay_sec: Pause between attempts.
        skip_patterns: ``fnmatch`` patterns of node names that are not dumped.
    """

    spin_time_sec: float = 5.0
    timeout_sec: float = 30.0
    attempts: int = 3
    retry_delay_sec: float = 2.0
    skip_patterns: Tuple[str, ...] = DEFAULT_SKIP_PATTERNS


@dataclass(frozen=True)
class DumpReport:
    """Outcome of dumping a graph.

    Attributes:
        nodes: Node names found, unique and sorted.
        dumped: Nodes whose parameters were written.
        skipped: Nodes that match a skip pattern.
        failed: Nodes that could not be dumped.
        duplicates: Names used by more than one node, which usually means a
            stale or foreign process shares the DDS domain.
        list_failed: Whether listing the nodes failed.
    """

    nodes: Tuple[str, ...]
    dumped: Tuple[str, ...]
    skipped: Tuple[str, ...]
    failed: Tuple[str, ...]
    duplicates: Tuple[str, ...] = ()
    list_failed: bool = False

    @property
    def ok(self) -> bool:
        """bool: Whether nodes were listed and every node not skipped was dumped."""
        return not self.list_failed and not self.failed

    def to_dict(self) -> Dict[str, Any]:
        """Convert the report to YAML- and JSON-serializable data.

        Returns:
            A mapping with one list per node category and ``list_failed``.
        """
        return {
            'nodes': list(self.nodes),
            'dumped': list(self.dumped),
            'skipped': list(self.skipped),
            'failed': list(self.failed),
            'duplicates': list(self.duplicates),
            'list_failed': self.list_failed,
        }


def node_file_name(node: str) -> str:
    """Map a node name to the name of its dump file.

    Args:
        node: Fully qualified node name, for example ``/local_costmap/local_costmap``.

    Returns:
        The name without its leading slash, other slashes replaced by ``__``,
        plus ``.yaml``, matching the golden parameter files.

    Raises:
        ValueError: If the name is empty.
    """
    stripped = node[1:] if node.startswith('/') else node
    if not stripped:
        raise ValueError(f'invalid node name {node!r}')
    return stripped.replace('/', '__') + '.yaml'


def should_skip(node: str, patterns: Sequence[str]) -> bool:
    """Check whether a node matches a skip pattern.

    Args:
        node: Fully qualified node name.
        patterns: ``fnmatch`` patterns.

    Returns:
        ``True`` if any pattern matches the whole name.
    """
    return any(fnmatch.fnmatchcase(node, pattern) for pattern in patterns)


def parse_node_list(output: str) -> Tuple[List[str], List[str]]:
    """Parse the output of ``ros2 node list``.

    Args:
        output: Standard output of the command.

    Returns:
        ``(names, duplicates)``: unique node names in sorted order, and the
        names listed more than once. Lines that are not node names, such as
        warnings, are ignored.
    """
    counts = Counter(line.strip() for line in output.splitlines() if line.strip().startswith('/'))
    names = sorted(counts)
    return names, [name for name in names if counts[name] > 1]


def default_runner(settings: DumpSettings) -> manifest.CommandRunner:
    """Build a command runner with the call timeout of the settings.

    Args:
        settings: Dump settings.

    Returns:
        :func:`nav_bringup.recorder.manifest.run_command` bound to the timeout.
    """
    return functools.partial(manifest.run_command, timeout_sec=settings.timeout_sec)


def dump_graph(
    output_dir: Union[str, Path],
    runner: Optional[manifest.CommandRunner] = None,
    settings: DumpSettings = DumpSettings(),
    sleep: Callable[[float], None] = time.sleep,
) -> DumpReport:
    """Dump the parameters of every node of the current ROS graph.

    Writes one ``<node file name>.yaml`` per dumped node, ``nodes.txt`` with
    the listed names, and ``dump_report.yaml`` with the report.

    Args:
        output_dir: Destination directory; created if missing.
        runner: Runs ``ros2`` commands; defaults to :func:`default_runner`.
        settings: Discovery, timeout, retry and skip settings.
        sleep: Pause function between attempts, replaceable in tests.

    Returns:
        What was dumped, skipped and failed.
    """
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    run = runner if runner is not None else default_runner(settings)
    spin_time = f'{settings.spin_time_sec:g}'
    code, listing = run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', spin_time])
    if code != 0:
        report = DumpReport(nodes=(), dumped=(), skipped=(), failed=(), list_failed=True)
    else:
        names, duplicates = parse_node_list(listing)
        (output / NODES_FILE_NAME).write_text(
            ''.join(f'{name}\n' for name in names), encoding='utf-8')
        dumped: List[str] = []
        skipped: List[str] = []
        failed: List[str] = []
        for name in names:
            if should_skip(name, settings.skip_patterns):
                skipped.append(name)
            elif _dump_node(name, output / node_file_name(name), run, settings, sleep):
                dumped.append(name)
            else:
                failed.append(name)
        report = DumpReport(
            nodes=tuple(names),
            dumped=tuple(dumped),
            skipped=tuple(skipped),
            failed=tuple(failed),
            duplicates=tuple(duplicates),
        )
    with (output / REPORT_FILE_NAME).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(report.to_dict(), handle, sort_keys=False)
    return report


def _dump_node(
    node: str,
    destination: Path,
    run: manifest.CommandRunner,
    settings: DumpSettings,
    sleep: Callable[[float], None],
) -> bool:
    command = ['ros2', 'param', 'dump', '--no-daemon', '--spin-time',
               f'{settings.spin_time_sec:g}', node]
    for attempt in range(1, settings.attempts + 1):
        code, output = run(command)
        if code == 0 and output.strip():
            destination.write_text(output, encoding='utf-8')
            return True
        if attempt < settings.attempts:
            sleep(settings.retry_delay_sec)
    return False
