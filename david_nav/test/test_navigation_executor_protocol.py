import pytest

from david_nav.navigation_protocol import (
    CommandParseError,
    build_arrived_event,
    build_error_event,
    build_mark_poi_ack,
    build_mark_poi_error,
    build_mark_poi_success_event,
    build_progress_event,
    parse_command,
)
from david_nav.navigation_types import PoseRecord, StoredPoi


def test_parse_navigate_to_command_accepts_go2w_flat_schema():
    command = parse_command(
        {
            "action": "navigate_to",
            "request_id": "req_001",
            "sub_id": 7,
            "target_id": "POI_001",
        }
    )

    assert command.action == "navigate_to"
    assert command.request_id == "req_001"
    assert command.sub_id == 7
    assert command.target_id == "POI_001"


def test_parse_navigate_to_ignores_v2_target_pose_and_uses_target_id():
    # v1(2026-07-22 回退定稿):target_pose/label 是已废弃的 v2 字段,
    # 即使残留也要忽略,只认 target_id。
    command = parse_command(
        {
            "action": "navigate_to",
            "request_id": "nav_ab12cd34ef56",
            "sub_id": 0,
            "target_id": "POI_012",
            "target_pose": {"x": 9.52, "y": -8.82, "theta": 0.0},
            "label": "前台",
        }
    )

    assert command.action == "navigate_to"
    assert command.target_id == "POI_012"
    assert not hasattr(command, "x")


def test_parse_navigate_to_without_target_id_raises_command_parse_error():
    # v1 checklist:target_id 缺字段/类型不对 → on_error,不启车。
    # 只带 target_pose 的旧 v2 消息也落到这条路径。
    with pytest.raises(CommandParseError) as exc_info:
        parse_command(
            {
                "action": "navigate_to",
                "request_id": "nav_x",
                "sub_id": 0,
                "target_pose": {"x": 1.0, "y": 2.0, "theta": -1.5},
            }
        )

    assert exc_info.value.request_id == "nav_x"
    assert exc_info.value.sub_id == 0


def test_parse_navigate_to_with_non_string_target_id_raises_command_parse_error():
    with pytest.raises(CommandParseError):
        parse_command(
            {
                "action": "navigate_to",
                "request_id": "nav_y",
                "sub_id": 0,
                "target_id": 12,
            }
        )


def test_parse_mark_current_poi_requires_flat_poi_object():
    command = parse_command(
        {
            "action": "mark_current_poi",
            "request_id": "req_mark",
            "sub_id": 1,
            "poi": {"id": "POI_006", "name": "会议室门口"},
        }
    )

    assert command.action == "mark_current_poi"
    assert command.poi_id == "POI_006"
    assert command.poi_name == "会议室门口"


def test_parse_abort_navigation_command_uses_flat_schema():
    command = parse_command(
        {
            "action": "abort_navigation",
            "request_id": "req_abort",
        }
    )

    assert command.action == "abort_navigation"
    assert command.request_id == "req_abort"


def test_parse_update_poi_list_command_uses_top_level_poi_list():
    command = parse_command(
        {
            "action": "update_poi_list",
            "version": 3,
            "poi_list": [{"id": "POI_001", "name": "前台"}],
        }
    )

    assert command.action == "update_poi_list"
    assert command.version == 3
    assert command.poi_list == [{"id": "POI_001", "name": "前台"}]


def test_build_progress_event_rounds_remaining_distance_to_three_decimals():
    payload = build_progress_event("req_001", 7, 1.23456, "running")

    assert payload == {
        "event_type": "on_progress",
        "request_id": "req_001",
        "sub_id": 7,
        "remaining_distance": 1.235,
        "status": "running",
    }


def test_build_mark_poi_success_event_only_returns_semantic_fields():
    poi = StoredPoi(
        poi_id="POI_006",
        name="会议室门口",
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        slam_session_id="session_a",
    )

    payload = build_mark_poi_success_event("req_mark", 1, poi)

    assert payload == {
        "event_type": "on_mark_poi_success",
        "request_id": "req_mark",
        "sub_id": 1,
        "poi": {
            "id": "POI_006",
            "name": "会议室门口",
        },
    }


def test_build_arrived_event_uses_navigation_event_schema():
    payload = build_arrived_event("req_001", 7)

    assert payload == {
        "event_type": "on_arrived",
        "request_id": "req_001",
        "sub_id": 7,
    }


def test_build_error_event_uses_navigation_event_schema():
    payload = build_error_event("req_001", 7, "路径被堵")

    assert payload == {
        "event_type": "on_error",
        "request_id": "req_001",
        "sub_id": 7,
        "error_message": "路径被堵",
    }


def test_build_mark_poi_ack_uses_flat_event_schema():
    payload = build_mark_poi_ack("req_mark", 2)

    assert payload == {
        "event_type": "on_mark_poi_ack",
        "request_id": "req_mark",
        "sub_id": 2,
        "status": "accepted",
    }


def test_build_mark_poi_error_uses_flat_event_schema():
    payload = build_mark_poi_error("req_mark", 2, "当前位置定位失败")

    assert payload == {
        "event_type": "on_mark_poi_error",
        "request_id": "req_mark",
        "sub_id": 2,
        "error_message": "当前位置定位失败",
    }
