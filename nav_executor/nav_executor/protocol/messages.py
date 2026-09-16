"""Legacy WS v1 command values and contextual parse errors, independent of I/O."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union


class NavExecutorError(Exception):
    """Base for executor-owned protocol failures, separate from public task errors."""


class ProtocolError(NavExecutorError, ValueError):
    """Retain legacy ValueError compatibility and recoverable wire request context.

    Attributes:
        request_id: 可恢复的请求标识；无法恢复时为 None。
        sub_id: 尽力恢复的子任务编号，解码失败时沿用零值策略。
    """

    def __init__(self, message: str, request_id: str | None = None, sub_id: int = 0) -> None:
        """Retain recoverable request context with the original error text.

        Args:
            message: Human-readable protocol failure.
            request_id: Recovered request identity, or None.
            sub_id: Recovered subtask number, defaulting to zero at decoding.
        """
        super().__init__(message)
        self.request_id = request_id
        self.sub_id = sub_id


class CommandParseError(ProtocolError):
    """Compatibility base for contextual malformed-command errors at the old import path."""


class MalformedCommandError(CommandParseError):
    """A missing field or invalid shape/conversion; reply only with recoverable context."""


class UnsupportedActionError(ProtocolError):
    """An unknown action to log and ignore, without inventing a response frame."""


@dataclass(frozen=True)
class NavigateToCommand:
    """A v1 navigation request referring only to a stored POI identifier.

    Attributes:
        action: 命令动作名 navigate_to。
        request_id: 上游请求标识，用于关联事件。
        sub_id: 上游子任务编号。
        target_id: 已有 POI 标识，不携带直接导航坐标。
    """

    action: str
    request_id: str
    sub_id: int
    target_id: str


@dataclass(frozen=True)
class MarkCurrentPoiCommand:
    """A request to capture the current pose as a named POI.

    Attributes:
        action: 命令动作名 mark_current_poi。
        poi_id: 要标记的 POI 标识。
        poi_name: POI 显示名称，可为空字符串。
        request_id: 上游请求标识，用于关联事件。
        sub_id: 上游子任务编号。
    """

    action: str
    request_id: str
    sub_id: int
    poi_id: str
    poi_name: str


@dataclass(frozen=True)
class AbortNavigationCommand:
    """A cancellation request scoped to one navigation request identifier.

    Attributes:
        action: 命令动作名 abort_navigation。
        request_id: 待取消的匹配请求标识。
    """

    action: str
    request_id: str


@dataclass(frozen=True)
class UpdatePoiListCommand:
    """A versioned semantic directory update, preserving the legacy list payload.

    Attributes:
        action: 命令动作名 update_poi_list。
        poi_list: 保留原输入值的目录项列表，只更新已有 POI 名称。
        version: 已解码的目录版本号；当前不参与版本冲突裁决。
    """

    action: str
    version: int
    poi_list: list[dict]


Command = Union[
    NavigateToCommand,
    MarkCurrentPoiCommand,
    AbortNavigationCommand,
    UpdatePoiListCommand,
]
