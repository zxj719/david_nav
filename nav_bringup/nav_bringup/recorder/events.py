"""Structured run events stored as JSON lines.

Events are the machine readable timeline of a run: task lifecycle, recovery
and escape triggers, process exits, check results. Each line of
``events.jsonl`` is one JSON object with the keys ``stamp``, ``source``,
``type``, ``severity`` and ``data`` (refactor plan §6.4, §13.2).
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import json
from pathlib import Path
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple, Union

SEVERITIES = ('info', 'warn', 'error')


@dataclass(frozen=True)
class Event:
    """One run event.

    Attributes:
        stamp: Wall-clock time in seconds since the epoch.
        source: Component that produced the event, for example ``navrun``.
        event_type: Event type, for example ``escape_triggered``. Stored under
            the JSON key ``type``.
        severity: One of :data:`SEVERITIES`.
        data: JSON-serializable details.
    """

    stamp: float
    source: str
    event_type: str
    severity: str = 'info'
    data: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate the severity.

        Raises:
            ValueError: If ``severity`` is not one of :data:`SEVERITIES`.
        """
        if self.severity not in SEVERITIES:
            raise ValueError(f'invalid severity {self.severity!r}')

    def to_json(self) -> str:
        """Serialize the event as one JSON line without a trailing newline.

        Returns:
            The JSON text; non-ASCII characters are kept as-is.
        """
        payload: Dict[str, Any] = {
            'stamp': round(self.stamp, 6),
            'source': self.source,
            'type': self.event_type,
            'severity': self.severity,
            'data': dict(self.data),
        }
        return json.dumps(payload, ensure_ascii=False)

    @classmethod
    def from_json(cls, line: str) -> 'Event':
        """Parse one JSON line.

        Args:
            line: Text produced by :meth:`to_json`.

        Returns:
            The parsed event.

        Raises:
            ValueError: If the line is not a valid event.
        """
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f'invalid JSON: {error}') from error
        if not isinstance(payload, dict):
            raise ValueError('an event must be a JSON object')
        try:
            return cls(
                stamp=float(payload['stamp']),
                source=str(payload['source']),
                event_type=str(payload['type']),
                severity=str(payload.get('severity', 'info')),
                data=dict(payload.get('data') or {}),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'invalid event: {error}') from error


class EventLog:
    """Append-only ``events.jsonl`` writer that is safe to share between threads."""

    def __init__(self, path: Union[str, Path], clock: Callable[[], float] = time.time) -> None:
        """Create a writer.

        Args:
            path: The ``events.jsonl`` file; its directory must exist.
            clock: Time source, replaceable in tests.
        """
        self._path = Path(path)
        self._clock = clock
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        """Path: The events file."""
        return self._path

    def append(
        self,
        source: str,
        event_type: str,
        severity: str = 'info',
        data: Optional[Mapping[str, Any]] = None,
    ) -> Event:
        """Record an event.

        Args:
            source: Producing component.
            event_type: Event type.
            severity: One of :data:`SEVERITIES`.
            data: JSON-serializable details.

        Returns:
            The recorded event.

        Raises:
            ValueError: If ``severity`` is not one of :data:`SEVERITIES`.
        """
        if severity not in SEVERITIES:
            raise ValueError(f'invalid severity {severity!r}')
        event = Event(self._clock(), source, event_type, severity, dict(data or {}))
        line = event.to_json() + '\n'
        with self._lock:
            with self._path.open('a', encoding='utf-8') as handle:
                handle.write(line)
        return event


def read_events(path: Union[str, Path]) -> Tuple[List[Event], int]:
    """Read an ``events.jsonl`` file.

    Args:
        path: The events file. A missing file yields no events.

    Returns:
        ``(events, skipped)``: the valid events in file order and the number
        of non-empty lines that could not be parsed.
    """
    file_path = Path(path)
    if not file_path.is_file():
        return [], 0
    events: List[Event] = []
    skipped = 0
    with file_path.open('r', encoding='utf-8') as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                events.append(Event.from_json(line))
            except ValueError:
                skipped += 1
    return events, skipped
