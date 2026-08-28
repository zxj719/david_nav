from __future__ import annotations

from dataclasses import dataclass
from typing import Union

from david_nav.navigation_types import StoredPoi


class CommandParseError(ValueError):
    """navigate_to 等命令字段缺失/类型错。

    v1 checklist:"target_id 缺字段/类型不对 → 发 on_error,不要启车"。
    带上 request_id/sub_id(尽力从 payload 提取),core 据此构造 on_error;
    request_id 也拿不到时 core 只能记日志丢弃。
    """

    def __init__(self, message: str, request_id: str | None, sub_id: int):
        super().__init__(message)
        self.request_id = request_id
        self.sub_id = sub_id


@dataclass(frozen=True)
class NavigateToCommand:
    action: str
    request_id: str
    sub_id: int
    target_id: str


@dataclass(frozen=True)
class MarkCurrentPoiCommand:
    action: str
    request_id: str
    sub_id: int
    poi_id: str
    poi_name: str


@dataclass(frozen=True)
class AbortNavigationCommand:
    action: str
    request_id: str


@dataclass(frozen=True)
class UpdatePoiListCommand:
    action: str
    version: int
    poi_list: list[dict]


Command = Union[
    NavigateToCommand,
    MarkCurrentPoiCommand,
    AbortNavigationCommand,
    UpdatePoiListCommand,
]


def _extract_request_context(payload: dict) -> tuple[str | None, int]:
    request_id = payload.get("request_id")
    request_id = str(request_id).strip() if request_id is not None else None
    try:
        sub_id = int(payload.get("sub_id", 0))
    except (TypeError, ValueError):
        sub_id = 0
    return request_id or None, sub_id


def parse_command(payload: dict) -> Command:
    action = str(payload.get("action", "")).strip()
    if action == "navigate_to":
        # nav 协议 v1(2026-07-22 回退定稿):只认 target_id(POI id 字符串)。
        # v2 的 target_pose/label 字段按协议要求直接忽略——即使还带着,也
        # 不再走坐标直达路径。target_id 缺失/类型错 → CommandParseError,
        # core 回 on_error,不启车。
        request_id, sub_id = _extract_request_context(payload)
        if request_id is None:
            raise CommandParseError("navigate_to 缺少 request_id", None, sub_id)
        target_id = payload.get("target_id")
        if not isinstance(target_id, str) or not target_id.strip():
            raise CommandParseError(
                "navigate_to 缺少有效 target_id(v1 只认 POI id,target_pose 已废弃)",
                request_id,
                sub_id,
            )
        return NavigateToCommand(
            action="navigate_to",
            request_id=request_id,
            sub_id=sub_id,
            target_id=target_id.strip(),
        )
    if action == "mark_current_poi":
        poi = payload["poi"]
        return MarkCurrentPoiCommand(
            action="mark_current_poi",
            request_id=str(payload["request_id"]).strip(),
            sub_id=int(payload["sub_id"]),
            poi_id=str(poi["id"]).strip(),
            poi_name=str(poi["name"]).strip(),
        )
    if action == "abort_navigation":
        return AbortNavigationCommand(
            action="abort_navigation",
            request_id=str(payload["request_id"]).strip(),
        )
    if action == "update_poi_list":
        return UpdatePoiListCommand(
            action="update_poi_list",
            version=int(payload.get("version", 0)),
            poi_list=list(payload.get("poi_list", [])),
        )
    raise ValueError(f"unsupported action: {action}")


def build_progress_event(
    request_id: str,
    sub_id: int,
    remaining_distance: float,
    status: str,
) -> dict:
    return {
        "event_type": "on_progress",
        "request_id": request_id,
        "sub_id": sub_id,
        "remaining_distance": round(float(remaining_distance), 3),
        "status": status,
    }


def build_mark_poi_success_event(
    request_id: str,
    sub_id: int,
    poi: StoredPoi,
) -> dict:
    return {
        "event_type": "on_mark_poi_success",
        "request_id": request_id,
        "sub_id": sub_id,
        "poi": {
            "id": poi.poi_id,
            "name": poi.name,
        },
    }


def build_arrived_event(request_id: str, sub_id: int) -> dict:
    return {
        "event_type": "on_arrived",
        "request_id": request_id,
        "sub_id": sub_id,
    }


def build_error_event(request_id: str, sub_id: int, error_message: str) -> dict:
    return {
        "event_type": "on_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }


def build_mark_poi_ack(request_id: str, sub_id: int) -> dict:
    return {
        "event_type": "on_mark_poi_ack",
        "request_id": request_id,
        "sub_id": sub_id,
        "status": "accepted",
    }


def build_mark_poi_error(request_id: str, sub_id: int, error_message: str) -> dict:
    return {
        "event_type": "on_mark_poi_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }
