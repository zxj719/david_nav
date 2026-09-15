"""Decode legacy commands and build ordered event mappings without I/O."""

from __future__ import annotations

from nav_tasks.api import StoredPoi

from nav_executor.protocol.messages import (
    AbortNavigationCommand, Command, CommandParseError, MarkCurrentPoiCommand,
    NavigateToCommand, UpdatePoiListCommand,
)


def _extract_request_context(payload: dict) -> tuple[str | None, int]:
    request_id = payload.get("request_id")
    request_id = str(request_id).strip() if request_id is not None else None
    try:
        sub_id = int(payload.get("sub_id", 0))
    except (TypeError, ValueError):
        sub_id = 0
    return request_id or None, sub_id


def parse_command(payload: dict) -> Command:
    """Decode a legacy command without changing malformed-frame behavior.

    Args:
        payload: Already decoded JSON mapping.

    Returns:
        The corresponding command value object.

    Raises:
        CommandParseError: A navigate_to request lacks required context or target.
        ValueError: An action is unsupported or a legacy numeric conversion fails.
        KeyError: A non-navigation command lacks a required field.
        TypeError: A legacy field cannot be converted to its expected type.
    """
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
    """Build the ordered progress mapping with legacy three-decimal rounding.

    Args:
        request_id: Request correlation identifier.
        sub_id: Subtask correlation number.
        remaining_distance: Remaining distance in meters.
        status: Legacy running/paused status string.

    Returns:
        Ordered protocol mapping, ready for unchanged JSON serialization.
    """
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
    """Build the successful mark acknowledgement using the stored POI identity.

    Args:
        request_id: Request correlation identifier.
        sub_id: Subtask correlation number.
        poi: Stored POI providing its identifier and name.

    Returns:
        Ordered legacy on_mark_poi_success event mapping.
    """
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
    """Build the successful navigation terminal event.

    Args:
        request_id: Request correlation identifier.
        sub_id: Subtask correlation number.

    Returns:
        Ordered legacy on_arrived event mapping.
    """
    return {
        "event_type": "on_arrived",
        "request_id": request_id,
        "sub_id": sub_id,
    }


def build_error_event(request_id: str, sub_id: int, error_message: str) -> dict:
    """Build the navigation failure event without altering its error text.

    Args:
        request_id: Request correlation identifier.
        sub_id: Subtask correlation number.
        error_message: Failure detail converted with the legacy str operation.

    Returns:
        Ordered legacy on_error event mapping.
    """
    return {
        "event_type": "on_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }


def build_mark_poi_ack(request_id: str, sub_id: int) -> dict:
    """Build the immediate acknowledgement before pose capture completes.

    Args:
        request_id: Request correlation identifier.
        sub_id: Subtask correlation number.

    Returns:
        Ordered accepted acknowledgement mapping.
    """
    return {
        "event_type": "on_mark_poi_ack",
        "request_id": request_id,
        "sub_id": sub_id,
        "status": "accepted",
    }


def build_mark_poi_error(request_id: str, sub_id: int, error_message: str) -> dict:
    """Build a capture/storage failure event with the existing wire field order.

    Args:
        request_id: Request correlation identifier.
        sub_id: Subtask correlation number.
        error_message: Failure detail converted with the legacy str operation.

    Returns:
        Ordered legacy on_mark_poi_error event mapping.
    """
    return {
        "event_type": "on_mark_poi_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }
