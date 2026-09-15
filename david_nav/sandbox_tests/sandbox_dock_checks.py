"""Sandbox checks: dock_to_pose action and executor routing of dock POIs (alias 0, 2026-09-15).

Not collected by colcon test (the file name does not match test_*.py). Run
``david_nav/sandbox_tests/run_sandbox_checks.sh`` in DDS domain 99 while no
navigation stack runs. The checks start fake Nav2 action servers with a fake
map->base_footprint TF, the real dock_navigator, and the real
navigation_executor against a fake agent server on 127.0.0.1. Nothing
publishes velocity commands.
"""

import asyncio
import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient

HERE = Path(__file__).resolve().parent
PACKAGE_DIR = HERE.parent
GOAL = (1.0, 2.0, 0.5)
APPROACH_DIST = 0.5
AGENT_PORT = 18101


def _fake_nav2(log_path, delay_sec, map_mode='free'):
    return ['python3', str(HERE / 'fake_nav2_servers.py'), '--log', str(log_path),
            '--delay', str(delay_sec), '--x', str(GOAL[0]), '--y', str(GOAL[1]),
            '--yaw', str(GOAL[2]), '--map-mode', map_mode]


def _dock_navigator():
    return ['ros2', 'run', 'david_nav', 'dock_navigator', '--ros-args',
            '-p', 'goal_topic:=/dock_goal', '-p', 'approach_dist:=%s' % APPROACH_DIST,
            '-p', 'xy_ok:=0.10', '-p', 'yaw_ok:=0.4363', '-p', 'max_yaw_retries:=0',
            '-p', 'robot_base_frame:=base_footprint', '-p', 'global_frame:=map']


def _goal():
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = GOAL[0]
    goal.pose.pose.position.y = GOAL[1]
    goal.pose.pose.orientation.z = math.sin(GOAL[2] / 2.0)
    goal.pose.pose.orientation.w = math.cos(GOAL[2] / 2.0)
    return goal


def _spin_until(node, future, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def _spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def _stage_goals(log_path):
    if not log_path.exists():
        return []
    return [tuple(float(value) for value in line.split()[1:])
            for line in log_path.read_text(encoding='utf-8').splitlines()
            if line.startswith('goal')]


@pytest.fixture
def ros_node():
    """Provide a ROS node for action clients in the sandbox domain."""
    rclpy.init()
    node = rclpy.create_node('sandbox_dock_client')
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def _send_dock_goal(node, feedback):
    client = ActionClient(node, NavigateToPose, 'dock_to_pose')
    assert client.wait_for_server(timeout_sec=30.0), 'dock_to_pose not available'
    # Let the fixed TF and transient-local map reach the separate dock process.
    _spin_for(node, 1.0)
    sent = client.send_goal_async(
        _goal(), feedback_callback=lambda msg: feedback.append(msg.feedback.distance_remaining))
    assert _spin_until(node, sent, 10.0)
    return client, sent.result()


def test_dock_action_runs_both_stages_and_succeeds(sessions, ros_node, tmp_path):
    """The action sends the approach and final goals and reports SUCCEEDED."""
    log = tmp_path / 'nav2.log'
    sessions.start('fake_nav2', _fake_nav2(log, 0.5))
    sessions.start('dock_navigator', _dock_navigator())
    feedback = []

    client, handle = _send_dock_goal(ros_node, feedback)
    assert handle.accepted
    result = handle.get_result_async()
    assert _spin_until(ros_node, result, 60.0)

    assert result.result().status == GoalStatus.STATUS_SUCCEEDED
    approach = (GOAL[0] - APPROACH_DIST * math.cos(GOAL[2]),
                GOAL[1] - APPROACH_DIST * math.sin(GOAL[2]))
    goals = _stage_goals(log)
    assert len(goals) == 2
    assert goals[0] == pytest.approx(approach, abs=0.002)
    assert goals[1] == pytest.approx(GOAL[:2], abs=0.002)
    assert feedback
    assert max(feedback) == pytest.approx(1.0 + APPROACH_DIST, abs=0.01)
    client.destroy()


@pytest.mark.parametrize('map_mode', ['occupied_approach', 'none'])
def test_dock_rejects_an_occupied_or_unverified_approach(sessions, ros_node, tmp_path, map_mode):
    """An occupied footprint corner or missing map aborts without any Nav2 stage goal."""
    log = tmp_path / 'nav2.log'
    sessions.start('fake_nav2', _fake_nav2(log, 0.5, map_mode))
    sessions.start('dock_navigator', _dock_navigator())
    client, handle = _send_dock_goal(ros_node, [])
    assert handle.accepted
    result = handle.get_result_async()
    assert _spin_until(ros_node, result, 15.0)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert _stage_goals(log) == []
    client.destroy()


def test_dock_action_cancel_stops_the_current_stage(sessions, ros_node, tmp_path):
    """Canceling the action cancels the in-flight stage goal and sends no further stage."""
    log = tmp_path / 'nav2.log'
    sessions.start('fake_nav2', _fake_nav2(log, 30.0))
    sessions.start('dock_navigator', _dock_navigator())

    client, handle = _send_dock_goal(ros_node, [])
    assert handle.accepted
    _spin_for(ros_node, 2.0)
    cancel = handle.cancel_goal_async()
    assert _spin_until(ros_node, cancel, 10.0)
    result = handle.get_result_async()
    assert _spin_until(ros_node, result, 30.0)
    _spin_for(ros_node, 1.0)

    assert result.result().status == GoalStatus.STATUS_CANCELED
    lines = log.read_text(encoding='utf-8').splitlines()
    assert len(_stage_goals(log)) == 1
    assert 'canceled' in lines
    client.destroy()


async def _agent_session(sessions, tmp_path, poses):
    import websockets

    frames = []
    finished = asyncio.Event()

    async def handler(websocket):
        await asyncio.sleep(3.0)
        await websocket.send(json.dumps({
            'action': 'navigate_to', 'request_id': 'dock-1', 'sub_id': 1,
            'target_id': 'POI_017'}))
        async for raw in websocket:
            payload = json.loads(raw)
            if payload.get('type') == 'ping':
                await websocket.send(json.dumps({'type': 'pong'}))
                continue
            frames.append(payload)
            if payload.get('event_type') in ('on_arrived', 'on_error'):
                finished.set()
                return

    async with websockets.serve(handler, '127.0.0.1', AGENT_PORT):
        sessions.start('executor', [
            'python3', str(PACKAGE_DIR / 'david_nav' / 'navigation_executor.py'),
            '--server-uri', 'ws://127.0.0.1:%d' % AGENT_PORT,
            '--pose-file', str(poses),
            '--poi-store-file', str(tmp_path / 'poi_store.yaml'),
            '--dock-poi-id', 'POI_017'])
        await asyncio.wait_for(finished.wait(), timeout=120.0)
    return frames


def test_executor_docks_the_dock_poi_and_reports_arrival(sessions, tmp_path):
    """navigate_to POI_017 from the agent goes through two-stage docking and ends on_arrived."""
    log = tmp_path / 'nav2.log'
    sessions.start('fake_nav2', _fake_nav2(log, 0.5))
    sessions.start('dock_navigator', _dock_navigator())
    poses = tmp_path / 'named_poses.json'
    poses.write_text(json.dumps([{
        'id': 'POI_017', 'name': 'POI_017', 'frame_id': 'map',
        'position': {'x': GOAL[0], 'y': GOAL[1], 'z': 0.0}, 'yaw': GOAL[2],
    }]), encoding='utf-8')
    time.sleep(5.0)

    frames = asyncio.run(_agent_session(sessions, tmp_path, poses))

    final = [frame for frame in frames if frame.get('event_type') in ('on_arrived', 'on_error')]
    assert final and final[-1]['event_type'] == 'on_arrived', frames
    assert final[-1]['request_id'] == 'dock-1'
    assert len(_stage_goals(log)) == 2
