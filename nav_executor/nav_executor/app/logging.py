"""Frozen outbound audit lines for the process-lifetime reliable outbox."""

import json
from typing import Callable

from nav_executor.adapters.transport.logging import log_line


class OutboundAudit:
    """Provide pre-send error and post-send terminal/replay audit callbacks."""

    def __init__(self, emit: Callable[[str], None] = log_line) -> None:
        """Inject the runtime logger without creating an outbox or transport.

        Args:
            emit: Synchronous unprefixed-line sink; default is the legacy logger.
        """
        self._emit = emit

    def on_sending(self, encoded: str) -> None:
        """Audit every attempted error frame before the transport send.

        Args:
            encoded: JSON object frozen by ReliableOutbox.put.
        """
        payload = json.loads(encoded)
        if payload.get('event_type') == 'on_error':
            self._emit(
                'NAV ERROR reported to server: '
                f"request_id={payload.get('request_id')} "
                f"sub_id={payload.get('sub_id')} "
                f"error_message={payload.get('error_message')}")

    def on_sent(self, encoded: str, resend: bool) -> None:
        """Audit successful terminal/replayed frames without flooding progress logs.

        Args:
            encoded: JSON object whose send has completed.
            resend: Whether an earlier drain retained this frame for replay.
        """
        payload = json.loads(encoded)
        event_type = payload.get('event_type') or payload.get('type')
        if resend or event_type in ('on_arrived', 'on_error'):
            tag = 'RESENT(断联重发)' if resend else 'sent'
            self._emit(
                f'OUT {tag}: {event_type} '
                f"request_id={payload.get('request_id')} sub_id={payload.get('sub_id')}")
