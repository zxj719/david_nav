"""Component load failures and container exits reap only the owned inert stack."""

import os
from pathlib import Path
import re
import signal
import sys
import time

import pytest

from conftest import require_sandbox
from nav_bringup.navrun.owned_process import OwnedProcess


@pytest.mark.parametrize('scenario', ['rejected', 'timeout', 'container_exit'])
def test_composition_fault_shuts_down_owned_launch(tmp_path, scenario):
    """A real load rejection/deadline or container death cannot leave its sibling alive."""
    require_sandbox()
    log = tmp_path / 'launch.log'
    owner = OwnedProcess([
        sys.executable, str(Path(__file__).with_name('composition_failure_fixture.py')),
        scenario], os.environ, log)
    try:
        deadline = time.monotonic() + 15
        while owner.poll() is None and time.monotonic() < deadline:
            lines = log.read_text()
            if scenario == 'container_exit':
                matches = re.findall(r'process started with pid \[(\d+)\]', lines)
                if len(matches) == 2:
                    # Both PIDs are from this fixture's launch-owned children.
                    container = re.search(
                        r'component_container_isolated[^\n]+process started with pid \[(\d+)\]',
                        lines)
                    assert container
                    os.kill(int(container.group(1)), signal.SIGTERM)
                    break
            time.sleep(0.05)
        deadline = time.monotonic() + 15
        while owner.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        lines = log.read_text()
        assert owner.poll() == (0 if scenario == 'container_exit' else 1), lines
        assert {'rejected': 'failed to load', 'timeout': 'load timed out',
                'container_exit': 'restart budget 0 exhausted'}[scenario] in lines
        pids = re.findall(r'process started with pid \[(\d+)\]', lines)
        assert len(pids) == 2, lines
        assert all(not Path('/proc', pid).exists() for pid in pids)
    finally:
        owner.close()
