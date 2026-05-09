from __future__ import annotations

from dataclasses import dataclass
from typing import Union

from g1_nav.navigation_types import StoredPoi


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


def parse_command(payload: dict) -> Command:
    action = str(payload.get("action", "")).strip()
    if action == "navigate_to":
        return NavigateToCommand(
            action="navigate_to",
            request_id=str(payload["request_id"]).strip(),
            sub_id=int(payload["sub_id"]),
            target_id=str(payload["target_id"]).strip(),
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
