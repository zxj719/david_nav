"""Sandbox integration checks for ``navrun doctor --live`` (PROGRESS P1.5b).

Not collected by ``colcon test``; run with ``run_sandbox_checks.sh``. Two
publishers share a command topic and a static transform provides map to
odom: the doctor must warn about the publishers, see one host, and find the
transform. No motion command is published; the command topic carries
strings.
"""

import json
import subprocess
import time

COMMAND_TOPIC = '/navrun_sandbox/command'


def test_doctor_live_reports_publishers_hosts_and_transform(sessions):
    """Duplicate publishers warn; one host and an existing transform pass."""
    for index in (1, 2):
        sessions.start(f'publisher_{index}', [
            'ros2', 'topic', 'pub', '-r', '1', COMMAND_TOPIC, 'std_msgs/msg/String',
            '{data: sandbox}'])
    sessions.start('map_to_odom', [
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '--frame-id', 'map', '--child-frame-id', 'odom'])
    time.sleep(4.0)

    completed = subprocess.run(
        ['ros2', 'run', 'nav_bringup', 'navrun', 'doctor', '--json', '--live',
         '--cmd-topic', COMMAND_TOPIC, '--watch-topic', COMMAND_TOPIC],
        capture_output=True, text=True, timeout=300, check=False)

    results = {item['name']: item for item in json.loads(completed.stdout)}
    assert results['command_publishers']['status'] == 'WARN', completed.stdout
    assert '2 个发布者' in results['command_publishers']['message']
    assert results['foreign_hosts']['status'] == 'OK', completed.stdout
    assert results['map_to_odom']['status'] == 'OK', completed.stdout
    assert results['ros2_daemon']['status'] in ('OK', 'SKIP')
