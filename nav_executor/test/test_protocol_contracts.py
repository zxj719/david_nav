"""Preserve command decoding and ordered WS v1 event mappings without I/O."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

from nav_executor.protocol import codec, messages
from nav_tasks.api import PoseRecord, StoredPoi


def test_command_dataclasses_and_contextual_parse_errors_are_preserved():
    """Old parsing values and exception context remain unchanged."""
    command = codec.parse_command({'action': ' navigate_to ', 'request_id': ' req ',
                                   'sub_id': '4', 'target_id': ' point '})
    assert command == messages.NavigateToCommand('navigate_to', 'req', 4, 'point')
    with pytest.raises(messages.CommandParseError) as caught:
        codec.parse_command({'action': 'navigate_to', 'request_id': 'req', 'sub_id': 'bad'})
    assert caught.value.request_id == 'req' and caught.value.sub_id == 0
    assert str(caught.value) == 'navigate_to 缺少有效 target_id(v1 只认 POI id,target_pose 已废弃)'


@pytest.mark.parametrize('payload,error', [
    ({'action': 'mark_current_poi'}, messages.MalformedCommandError),
    ({'action': 'abort_navigation'}, messages.MalformedCommandError),
    ({'action': 'update_poi_list', 'poi_list': None}, messages.MalformedCommandError),
    ({'action': 'unknown'}, messages.UnsupportedActionError),
])
def test_r1_decoder_errors_are_typed_without_changing_valid_wire_contracts(payload, error):
    """The explicit R1 repair normalizes malformed fields separately from unknown actions."""
    with pytest.raises(error):
        codec.parse_command(payload)


def test_event_order_rounding_and_chinese_strings_remain_byte_exact():
    """Wire field order and ensure_ascii=False are still compatible with the caller."""
    event = codec.build_progress_event('req', 4, 1.23456, 'running')
    assert json.dumps(event, ensure_ascii=False) == (
        '{"event_type": "on_progress", "request_id": "req", "sub_id": 4, '
        '"remaining_distance": 1.235, "status": "running"}')
    assert json.dumps(codec.build_error_event('req', 4, '目标点不存在: 点'),
                      ensure_ascii=False) == (
        '{"event_type": "on_error", "request_id": "req", "sub_id": 4, '
        '"error_message": "目标点不存在: 点"}')
    assert list(codec.build_arrived_event('req', 4)) == ['event_type', 'request_id', 'sub_id']
    pose = PoseRecord('fixture', 0.0, 0.0, 0.0, 0.0, 0.0)
    poi = StoredPoi('id', '名', pose, pose, 'session')
    assert codec.build_mark_poi_success_event('req', 4, poi)['poi'] == {'id': 'id', 'name': '名'}
    assert codec.build_mark_poi_ack('req', 4)['status'] == 'accepted'
    assert codec.build_mark_poi_error('req', 4, '错误')['error_message'] == '错误'


def test_protocol_imports_work_without_ros_or_site_packages():
    """The wire layer depends only on stdlib and public task values."""
    root = Path(__file__).resolve().parents[2]
    paths = [str(root / package) for package in ('nav_tasks', 'nav_executor')]
    code = ('import sys; sys.path[:0] = ' + repr(paths) + '; '
            'import nav_executor.protocol.codec; '
            'assert not any(n in sys.modules for n in ("rclpy", "yaml", "websockets"))')
    result = subprocess.run([sys.executable, '-I', '-S', '-c', code],
                            capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr
