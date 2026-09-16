"""Legacy WS v1 command values and contextual parse errors, independent of I/O."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union


class NavExecutorError(Exception):
    """Base for executor-owned protocol failures, separate from public task errors."""


class ProtocolError(NavExecutorError, ValueError):
    """Retain legacy ValueError compatibility and recoverable wire request context."""

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
