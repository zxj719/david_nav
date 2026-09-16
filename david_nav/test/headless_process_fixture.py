"""Fail-closed, non-ROS executables for the legacy headless shell tests."""

import json
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import time


def _root():
    return Path(os.environ['HEADLESS_TEST_ROOT'])


def _audit(role, args):
    with (_root() / 'calls.jsonl').open('a') as stream:
        stream.write(json.dumps([role, *args]) + '\n')


def _stat(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return int(fields[1]), int(fields[2])
    except (OSError, ValueError, IndexError):
        return None


def _owned(pid, root_pid):
    seen = set()
    while pid > 1 and pid not in seen:
        if pid == root_pid:
            return True
        seen.add(pid)
        state = _stat(pid)
        if state is None:
            return False
        pid = state[0]
    return False


def _pgrep(args):
    # Never forward -f or arbitrary patterns to the host process table. Only
    # descendant/group queries rooted in this exact live test shell are allowed.
    if len(args) != 2 or args[0] not in ('-P', '-g') or not args[1].isdigit():
        return 1
    marker = _root() / 'shell.pid'
    if not marker.exists():
        return 1
    root_pid = int(marker.read_text())
    if not _owned(os.getpid(), root_pid):
        return 1
    selected = int(args[1])
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        state = _stat(pid)
        if (pid != os.getpid() and state is not None
                and state[0 if args[0] == '-P' else 1] == selected
                and _owned(pid, root_pid)):
            found.append(pid)
    for pid in sorted(found):
        print(pid)
    return 0 if found else 1


def _idle(label):
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    (_root() / (label + '.pid')).write_text(str(os.getpid()))
    while True:
        time.sleep(0.05)


def _launch(args):
    if args[:3] != ['launch', 'david_nav', '2d_david_nav2_bringup.launch.py']:
        return 97
    # The detached grandchild must be found by the script, not killed as part
    # of its parent's process group. Keep its parent alive to reap it normally.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    (_root() / 'nav.pid').write_text(str(os.getpid()))
    child = subprocess.Popen(
        [sys.executable, __file__, 'detached'], start_new_session=True,
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return child.wait()


def _python(args):
    if not args:
        return 97
    expected = Path(os.environ['HEADLESS_TEST_PACKAGE']) / 'scripts'
    if Path(args[0]) == expected / 'navigation_headless_config.py':
        sys.argv = args
        runpy.run_path(args[0], run_name='__main__')
        return 0
    if Path(args[0]) == expected / 'wait_nav_ready.py':
        if os.environ['HEADLESS_TEST_STAGE'] == 'probe':
            _idle('probe')
        return 0
    return 97


def main() -> int:
    """Dispatch explicit test roles without host-command delegation.

    Return the replacement status, or a refusal for unsupported operations.
    """
    role, *args = sys.argv[1:]
    if os.environ.get('ROS_DOMAIN_ID') != '99':
        return 98
    _audit(role, args)
    if role == 'pgrep':
        return _pgrep(args)
    if role == 'ros2':
        return _launch(args)
    if role == 'python':
        return _python(args)
    if role in ('executor', 'detached'):
        _idle(role)
    return 97


if __name__ == '__main__':
    sys.exit(main())
