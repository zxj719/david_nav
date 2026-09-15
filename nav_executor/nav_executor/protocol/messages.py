"""Legacy WS v1 command values and contextual parse errors, independent of I/O."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union


class CommandParseError(ValueError):
    """Report missing or malformed navigate_to fields with recoverable request context.

    v1 checklist:"target_id 缺字段/类型不对 → 发 on_error,不要启车"。
    带上 request_id/sub_id(尽力从 payload 提取),core 据此构造 on_error;
    request_id 也拿不到时 core 只能记日志丢弃。
    """

    def __init__(self, message: str, request_id: str | None, sub_id: int) -> None:
        """Retain recoverable request context with the original error text.

        Args:
            message: Human-readable protocol failure.
            request_id: Recovered request identity, or None.
            sub_id: Recovered subtask number, defaulting to zero at decoding.
        """
        super().__init__(message)
        self.request_id = request_id
        self.sub_id = sub_id


@dataclass(frozen=True)
class NavigateToCommand:
    """A v1 navigation request referring only to a stored POI identifier."""

    action: str
    request_id: str
    sub_id: int
    target_id: str


@dataclass(frozen=True)
class MarkCurrentPoiCommand:
    """A request to capture the current pose as a named POI."""

    action: str
    request_id: str
    sub_id: int
    poi_id: str
    poi_name: str


@dataclass(frozen=True)
class AbortNavigationCommand:
    """A cancellation request scoped to one navigation request identifier."""

    action: str
    request_id: str


@dataclass(frozen=True)
class UpdatePoiListCommand:
    """A versioned semantic directory update, preserving the legacy list payload."""

    action: str
    version: int
    poi_list: list[dict]


Command = Union[
    NavigateToCommand,
    MarkCurrentPoiCommand,
    AbortNavigationCommand,
    UpdatePoiListCommand,
]
