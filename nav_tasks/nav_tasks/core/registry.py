"""Extensible motion-action decoding and handling without wire-format imports."""

from dataclasses import dataclass
from typing import Awaitable, Callable, Mapping

from nav_tasks.core.ports import FeedbackCallback
from nav_tasks.domain.errors import UnknownActionError
from nav_tasks.domain.tasks import NavOutcome


@dataclass(frozen=True)
class TaskContext:
    """Skill correlation and generation-fenced loop-thread feedback callback."""

    task_id: str
    sub_id: int
    generation: int
    feedback: FeedbackCallback


Decoder = Callable[[Mapping[str, object]], object]
Handler = Callable[[TaskContext, object], Awaitable[NavOutcome]]


class CommandRegistry:
    """Register task actions without modifying task ownership code."""

    def __init__(self) -> None:
        """Create an empty registry without registering any runtime skill."""
        self._entries: dict[str, tuple[Decoder, Handler]] = {}

    def register(self, action: str, decoder: Decoder, handler: Handler) -> None:
        """Register one action without silently overriding another handler.

        Args:
            action: Nonempty action name selected by an ingress adapter.
            decoder: Pure synchronous decoder returning a detached domain value.
            handler: Async motion skill using the single shared backend.

        Raises:
            ValueError: The action is empty, duplicated or has a noncallable member.
        """
        if not action or action in self._entries:
            raise ValueError(f'empty or duplicate task action: {action!r}')
        if not callable(decoder) or not callable(handler):
            raise ValueError('decoder and handler must be callable')
        self._entries[action] = (decoder, handler)

    def prepare(self, action: str, payload: Mapping[str, object]) -> tuple[Handler, object]:
        """Decode before admission so invalid requests do not preempt motion.

        Args:
            action: Registered task action.
            payload: Transport-independent values from the ingress adapter.

        Returns:
            Handler and decoded domain value.

        Raises:
            UnknownActionError: The action has not been registered.
        """
        if action not in self._entries:
            raise UnknownActionError(f'unknown task action: {action}')
        decoder, handler = self._entries[action]
        return handler, decoder(payload)
