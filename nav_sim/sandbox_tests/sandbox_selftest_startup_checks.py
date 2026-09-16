"""Actual namespaced Nav2 startup and teardown with zero-only private velocity probes."""

import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import websockets
import yaml

from nav_bringup.navcfg import schema
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.navrun.up_planning import discover_inputs, UpRequest
from nav_sim.selftest_scene import prepare_scene
from nav_sim.selftest_launch import close_fixture


def _guard():
    assert os.environ.get('ROS_DOMAIN_ID') == '99'
    ancestors = set()
    pid = os.getpid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        pid = int(fields[1])
    scan = subprocess.run([
        'pgrep', '-f', 'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|'
        'wheeltec_driver_node|navigation_executor|dock_navigator|rviz2|component_container',
    ], capture_output=True, text=True, check=False)
    assert scan.returncode in (0, 1)
    assert not ({int(value) for value in scan.stdout.split()} - ancestors), scan.stdout


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_private_nav2_parameters_tf_and_cleanup_without_goals(tmp_path, mode):
    """Use passive WS, parameter reads and an explicitly zero-valued adapter probe."""
    _guard()

    async def scenario():
        connected = asyncio.Event()

        async def peer(socket):
            connected.set()
            async for raw in socket:
                frame = json.loads(raw)
                assert frame.get('event_type') not in ('on_arrived', 'on_error')
                # No outbound messages: neither navigation commands nor fake successes.

        async with websockets.serve(peer, '127.0.0.1', 0) as server:
            root = tmp_path / 'run'
            robot = 'example_diffdrive'
            namespace = 'selftest_' + uuid.uuid4().hex
            request = UpRequest(mode, robot, 'selftest', root)
            inputs = discover_inputs(request, lambda name: Path(get_package_share_directory(name)))
            catalog = schema.load_catalog(schema.find_installed_schemas(
                schema.ament_prefixes(os.environ)))
            scene = prepare_scene(
                root, robot, mode, namespace, inputs, catalog,
                f'ws://127.0.0.1:{server.sockets[0].getsockname()[1]}')
            prefix = '/' + namespace + '/'
            context = Context()
            rclpy.init(context=context, args=[])
            node = Node('zero_only_selftest_observer', namespace='/' + namespace,
                        context=context, use_global_arguments=False, cli_args=[
                            '--ros-args', '-r', '/tf:=' + prefix + 'tf',
                            '-r', '/tf_static:=' + prefix + 'tf_static'])
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            commands = []
            subscription = node.create_subscription(Twist, prefix + 'cmd_vel', commands.append, 10)
            publisher = node.create_publisher(Twist, prefix + 'cmd_vel_smoothed', 10)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            client = node.create_client(GetParameters, prefix + 'controller_server/get_parameters')
            lifecycle = [node.create_client(GetState, prefix + name + '/get_state')
                         for name in scene.graph.lifecycle_nodes]
            child = None
            try:
                child = OwnedProcess([
                    sys.executable, '-m', 'nav_sim.selftest_launch', '--run-dir', str(root),
                    '--plan-sha256', scene.digest, '--start-verified-fixture',
                ], dict(os.environ, ROS_LOG_DIR=str(root / 'ros_logs')), root / 'launch.log')
                deadline = time.monotonic() + 90
                while not connected.is_set() and time.monotonic() < deadline:
                    assert child.poll() is None, (root / 'launch.log').read_text()
                    for _ in range(8):
                        executor.spin_once(timeout_sec=0)
                    await asyncio.sleep(0.02)
                assert connected.is_set(), (root / 'launch.log').read_text()
                # Action endpoint discovery alone can precede lifecycle activation.
                # Read every lifecycle state before claiming the complete stack is ready.
                for state_client in lifecycle:
                    active = False
                    while not active and time.monotonic() < deadline:
                        if state_client.service_is_ready():
                            pending_state = state_client.call_async(GetState.Request())
                            while not pending_state.done() and time.monotonic() < deadline:
                                for _ in range(8):
                                    executor.spin_once(timeout_sec=0)
                                await asyncio.sleep(0.02)
                            assert pending_state.done()
                            active = pending_state.result().current_state.id == 3
                        await asyncio.sleep(0.1)
                    assert active, (root / 'launch.log').read_text()
                assert client.service_is_ready()
                query = GetParameters.Request(names=['odom_topic', 'FollowPath.max_vel_x'])
                pending = client.call_async(query)
                deadline = time.monotonic() + 10
                while not pending.done() and time.monotonic() < deadline:
                    for _ in range(8):
                        executor.spin_once(timeout_sec=0)
                    await asyncio.sleep(0.02)
                assert pending.done()
                result = pending.result().values
                profile = yaml.safe_load((root / 'scene/robot.yaml').read_text())
                assert result[0].string_value == prefix + 'odom'
                assert result[1].double_value == profile['limits']['max_vel_x']
                assert buffer.can_transform(
                    profile['frames']['map'], profile['frames']['nav_base'], rclpy.time.Time())
                # The adapter intentionally remains silent until its first input.
                # S1 permits only zeros in domain 99: never construct a nonzero Twist.
                deadline = time.monotonic() + 5
                while not commands and time.monotonic() < deadline:
                    if publisher.get_subscription_count():
                        publisher.publish(Twist())
                    for _ in range(8):
                        executor.spin_once(timeout_sec=0)
                    await asyncio.sleep(0.05)
                assert commands, 'adapter did not publish an observable idle command'
                for command in commands:
                    assert (command.linear.x, command.linear.y, command.linear.z,
                            command.angular.x, command.angular.y, command.angular.z) == (0,) * 6
                events = [json.loads(line) for line in
                          (root / 'process_events.jsonl').read_text().splitlines()]
                assert not [entry for entry in events if entry['event'] == 'exit']
                for process in scene.graph.processes:
                    assert any(
                        entry['node'] == prefix + process.name if process.ros_node else
                        entry['name'].startswith(process.name + '-')
                        or entry['name'] == process.name for entry in events), process.name
            finally:
                try:
                    if child is not None:
                        await close_fixture(child)
                finally:
                    node.destroy_client(client)
                    for state_client in lifecycle:
                        node.destroy_client(state_client)
                    node.destroy_subscription(subscription)
                    node.destroy_publisher(publisher)
                    listener.unregister()
                    executor.remove_node(node)
                    executor.shutdown()
                    node.destroy_node()
                    context.shutdown()
            assert child.poll() is not None
            final_events = [json.loads(line) for line in
                            (root / 'process_events.jsonl').read_text().splitlines()]
            exits = [entry for entry in final_events if entry['event'] == 'exit']
            assert len(exits) == len(scene.graph.processes)
            assert all(entry['returncode'] in (0, 130, -signal.SIGINT) for entry in exits), exits

    asyncio.run(scenario())
