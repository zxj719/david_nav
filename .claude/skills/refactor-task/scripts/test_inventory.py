#!/usr/bin/env python3
"""Collect pytest node IDs per package and diff two inventories.

collect SRC OUT
    Run ``pytest --collect-only`` inside every package of SRC that has a test/ directory
    (run it in a shell that sourced the workspace under test, ROS_DOMAIN_ID=99).
diff BEFORE AFTER [--allow NAME ...] [--allow-file FILE]
    Report removed and added tests. A removed test is accepted when its function name or
    test file name is allowed, or when the same function still exists in the same package
    (moved/renamed file). Exit 1 if any removal is unexpected.
"""

import argparse
from collections import Counter
from pathlib import Path
import re
import subprocess
import sys


def _function(node_id):
    return re.sub(r'\[.*\]$', '', node_id.split('::')[-1])


def _file(node_id):
    return node_id.split('::')[0].rsplit('/', 1)[-1]


def _package(node_id):
    return node_id.split('/', 1)[0]


def collect(source, output):
    """Write '<package>/<node id>' lines for every collected test."""
    lines = []
    for package in sorted(path for path in Path(source).iterdir() if (path / 'test').is_dir()):
        result = subprocess.run(
            [sys.executable, '-m', 'pytest', '--collect-only', '-q', '-p', 'no:cacheprovider',
             'test'], cwd=package, capture_output=True, text=True, timeout=600)
        found = [f'{package.name}/{line.strip()}' for line in result.stdout.splitlines()
                 if '::' in line]
        if result.returncode == 5:
            print(f'note: {package.name}: no pytest tests (C++/linters only)', file=sys.stderr)
        elif result.returncode != 0 or not found:
            print(f'warning: {package.name}: pytest collection exit {result.returncode}, '
                  f'{len(found)} tests', file=sys.stderr)
        lines.extend(found)
    Path(output).write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f'{len(lines)} tests -> {output}')


def _load(path):
    return {line.strip() for line in Path(path).read_text(encoding='utf-8').splitlines()
            if '::' in line}


def diff(before_path, after_path, allowed):
    """Print the comparison and return the number of unexpected removals."""
    before, after = _load(before_path), _load(after_path)
    removed, added = sorted(before - after), sorted(after - before)
    still_there = {(_package(node), _function(node)) for node in after}
    planned, moved, unexpected = Counter(), [], []
    for node in removed:
        if _function(node) in allowed or _file(node) in allowed:
            planned[_file(node)] += 1
        elif (_package(node), _function(node)) in still_there:
            moved.append(node)
        else:
            unexpected.append(node)
    print(f'before {len(before)}  after {len(after)}  removed {len(removed)}  '
          f'added {len(added)}')
    print(f'planned removals: {sum(planned.values())} in {len(planned)} files')
    print(f'moved (same function, same package): {len(moved)}')
    for node in moved:
        print(f'  moved   {node}')
    print(f'UNEXPECTED removals: {len(unexpected)}')
    for node in unexpected:
        print(f'  REMOVED {node}')
    for name in sorted({f'{_package(n)}/{_file(n)}::{_function(n)}' for n in added}):
        print(f'  added   {name}')
    return len(unexpected)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest='command', required=True)
    collect_parser = commands.add_parser('collect')
    collect_parser.add_argument('source')
    collect_parser.add_argument('output')
    diff_parser = commands.add_parser('diff')
    diff_parser.add_argument('before')
    diff_parser.add_argument('after')
    diff_parser.add_argument('--allow', nargs='*', default=[])
    diff_parser.add_argument('--allow-file')
    args = parser.parse_args()
    if args.command == 'collect':
        collect(args.source, args.output)
        return 0
    allowed = set(args.allow)
    if args.allow_file:
        for line in Path(args.allow_file).read_text(encoding='utf-8').splitlines():
            line = line.split('#', 1)[0].strip()
            if line:
                allowed.add(line)
    return 1 if diff(args.before, args.after, allowed) else 0


if __name__ == '__main__':
    sys.exit(main())
