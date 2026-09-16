"""Process-lifetime ordered outbox retaining in-flight frames across reconnects."""

import asyncio
import json
from typing import Awaitable, Callable


class ReliableOutbox:
    """Retain pending serialized frames until a send successfully completes.

    Construct and use on the task event loop. No background tasks or sockets are
    created here. A runtime supervisor owns one drain coroutine per connection;
    reconnect keeps this outbox, not the failed drain. Ambiguous transport failure
    can cause replay: this is not an exactly-once or server-acknowledged protocol.
    """

    def __init__(
        self, *, on_sending: Callable[[str], None] | None = None,
        on_sent: Callable[[str, bool], None] | None = None,
    ) -> None:
        """Construct an empty outbox and optional audit callbacks.

        Args:
            on_sending: Called with encoded frame before each send attempt.
            on_sent: Called with encoded frame and replay flag after successful send.
        """
        self._loop = asyncio.get_running_loop()
        self._queue: asyncio.Queue[str] = asyncio.Queue()
        self._pending: str | None = None
        self._resend = False
        self._drain_lock = asyncio.Lock()
        self._on_sending = on_sending
        self._on_sent = on_sent

    @property
    def in_flight_count(self) -> int:
        """Number of frames retained from an unfinished send, for legacy reconnect logs."""
        self._check_loop()
        return int(self._pending is not None)

    @property
    def pending_count(self) -> int:
        """Number of queued frames including any in-flight send."""
        self._check_loop()
        return self._queue.qsize() + int(self._pending is not None)

    def put(self, frame: dict) -> None:
        """Freeze and enqueue one frame with the legacy JSON serialization.

        Args:
            frame: Ordered WS mapping; subsequent caller mutation is harmless.

        Raises:
            TypeError: The frame is not JSON serializable; nothing is enqueued.
        """
        self._check_loop()
        self._queue.put_nowait(json.dumps(frame, ensure_ascii=False))

    async def drain(self, send: Callable[[str], Awaitable[None]]) -> None:
        """Send in order until canceled or a transport/audit callback raises.

        One drainer owns the outbox at a time. A canceled or failed send leaves its
        frame pending, ahead of frames arriving later. Successful send removes the
        frame before the audit callback, so a logging error cannot resend it.

        Args:
            send: Awaitable transport send operation, normally websocket.send.

        Raises:
            asyncio.CancelledError: Caller canceled; an in-flight frame is retained.
            Exception: Send or audit callback failed; reconnect may call drain again.
        """
        self._check_loop()
        async with self._drain_lock:
            while True:
                if self._pending is None:
                    self._pending = await self._queue.get()
                    self._resend = False
                encoded = self._pending
                resend = self._resend
                self._resend = True
                if self._on_sending is not None:
                    self._on_sending(encoded)
                await send(encoded)
                self._pending = None
                self._resend = False
                if self._on_sent is not None:
                    self._on_sent(encoded, resend)

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('outbox used from a different event loop')
