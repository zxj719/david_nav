"""Record only a sandbox-owned text topic, never velocity or navigation traffic."""

from pathlib import Path
import time

import pytest
import rclpy
from std_msgs.msg import String
import yaml

from nav_bringup.navrun.recording import plan_recording


@pytest.mark.parametrize('mode', ['light', 'full'])
def test_planned_sqlite_recording_flushes_owned_bag(sessions, tmp_path, mode):
    """The exact planned CLI creates SQLite metadata; full also retains zstd compression."""
    profile = {'record': {
        'light_topics': ['/sandbox_record_probe'], 'full_topics': ['/sandbox_record_probe'],
        'full_compression': 'zstd', 'full_include_hidden': True,
        'full_include_unpublished': True}}
    (tmp_path / 'bag').mkdir()
    plan = plan_recording(profile, mode, tmp_path)
    rclpy.init(args=[])
    source = rclpy.create_node('sandbox_bag_source')
    publisher = source.create_publisher(String, '/sandbox_record_probe', 10)
    try:
        recorder = sessions.start('sqlite_recording', list(plan.argv))
        deadline = time.monotonic() + 12.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert recorder.poll() is None, (
                tmp_path / 'process_logs/sqlite_recording.log').read_text()
            rclpy.spin_once(source, timeout_sec=0.1)
        assert publisher.get_subscription_count() > 0
        for _ in range(15):
            publisher.publish(String(data='navigation recording probe; no command'))
            rclpy.spin_once(source, timeout_sec=0.05)
        assert sessions.stop(recorder) == 0
        metadata = yaml.safe_load((Path(plan.output) / 'metadata.yaml').read_text())
        info = metadata['rosbag2_bagfile_information']
        assert info['storage_identifier'] == 'sqlite3'
        assert info['message_count'] > 0
        assert {item['topic_metadata']['name'] for item in info['topics_with_message_count']} == {
            '/sandbox_record_probe'}
        if mode == 'full':
            assert info['compression_format'] == 'zstd'
            assert all(path.endswith('.zstd') for path in info['relative_file_paths'])
        else:
            assert info['compression_format'] == ''
    finally:
        source.destroy_node()
        rclpy.shutdown()
