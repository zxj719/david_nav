"""Installed CLI, private ROS host and real WS ping with no goals or velocity publication."""

import asyncio
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest
import websockets

from nav_bringup.navrun.owned_process import OwnedProcess


def _guard():
    assert os.environ.get('ROS_DOMAIN_ID') == '99', 'sandbox requires ROS_DOMAIN_ID=99'
    ancestors = set()
    pid = os.getpid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        tail = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        pid = int(tail[1])
    found = subprocess.run([
        'pgrep', '-f', 'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|'
        'wheeltec_driver_node|navigation_executor|dock_navigator|rviz2|component_container',
    ], text=True, stdout=subprocess.PIPE, check=False)
    if found.returncode not in (0, 1):
        pytest.fail('cannot verify user stack state')
    live = {int(value) for value in found.stdout.split()} - ancestors
    assert not live, f'refusing ROS sandbox while user stack exists: {sorted(live)}'


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_installed_executor_graceful_signal_without_navigation(tmp_path, signum):
    """A private child receives only WS ping and exits after its own resources are cleaned."""
    _guard()

    async def scenario():
        connected = asyncio.Event()
        received, errors = [], []

        async def server(socket):
            try:
                await socket.send('{"type":"ping"}')
                received.append(await asyncio.wait_for(socket.recv(), 5))
                connected.set()
                await socket.wait_closed()
            except Exception as error:
                errors.append(error)
                connected.set()

        async with websockets.serve(server, '127.0.0.1', 0) as listener:
            port = listener.sockets[0].getsockname()[1]
            executable = (Path(get_package_prefix('nav_executor')) /
                          'lib/nav_executor/navigation_executor')
            env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros_logs'))
            output = tmp_path / 'executor.log'
            child = OwnedProcess([
                str(executable), '--server-uri', f'ws://127.0.0.1:{port}',
                '--poi-store-file', str(tmp_path / 'pois.yaml'),
                '--ros-args', '-r', f'__node:=executor_sandbox_{os.getpid()}',
            ], env, output)
            try:
                await asyncio.wait_for(connected.wait(), 12)
                assert not errors and received == ['{"type": "pong"}']
                assert child.poll() is None
                os.kill(child.pid, signum)
                deadline = time.monotonic() + 10
                while child.poll() is None and time.monotonic() < deadline:
                    await asyncio.sleep(0.05)
                assert child.poll() == 128 + signum, output.read_text()
            finally:
                child.close(timeout_sec=3)
            text = output.read_text()
            assert 'effective config:' in text and 'websockets version=' in text
            assert f'shutdown requested: {signum.name}' in text
            assert 'Traceback' not in text and 'cleanup pending' not in text
            assert not (tmp_path / 'pois.yaml').exists()

    asyncio.run(scenario())
