"""Transport-independent requests, outcomes and immutable task observations."""

from dataclasses import dataclass
from enum import Enum
from typing import Mapping

from nav_tasks.domain.types import StoredPoi


class NavStatus(str, Enum):
    """Terminal result of one navigation attempt, not an asyncio cancellation."""

    SUCCEEDED = 'succeeded'
    CANCELED = 'canceled'
    FAILED = 'failed'


@dataclass(frozen=True)
class NavOutcome:
    """Terminal status and human-readable failure detail from a backend or skill."""

    status: NavStatus
    detail: str = ''

    @property
    def succeeded(self) -> bool:
        """Whether this attempt reached its target successfully."""
        return self.status == NavStatus.SUCCEEDED


@dataclass(frozen=True)
class NavGoal:
    """One navigation target and request identity used only for correlation."""

    poi: StoredPoi
    request_id: str
    sub_id: int = 0


@dataclass(frozen=True)
class ResolvedGoal:
    """A navigable POI and whether it differs from the original anchor."""

    poi: StoredPoi
    used_snap: bool


class PreemptPolicy(str, Enum):
    """Ingress-selected admission policy; implicit motion preemption is forbidden."""

    ALLOW = 'allow'
    REJECT = 'reject'


class TaskState(str, Enum):
    """Motion ownership state independent of transport connectivity."""

    IDLE = 'idle'
    STARTING = 'starting'
    NAVIGATING = 'navigating'
    ABORTING = 'aborting'
    ERROR = 'error'
    CLOSED = 'closed'


@dataclass(frozen=True)
class TaskRequest:
    """Motion request; adapters translate their wire values before admission.

    Payload decoding is synchronous and completes before an existing task is
    preempted. Decoders must return a detached immutable domain object. Directory
    updates and POI marking are administrative operations, not motion requests.
    """

    task_id: str
    action: str
    payload: Mapping[str, object]
    sub_id: int = 0
    preempt: PreemptPolicy = PreemptPolicy.REJECT


@dataclass(frozen=True)
class TaskSnapshot:
    """Read-only ownership observation; generations distinguish reused task IDs."""

    state: TaskState
    task_id: str | None = None
    sub_id: int = 0
    generation: int = 0
    remaining_distance: float = 0.0

    target_id: str | None = None
    phase: str | None = None


class EventKind(str, Enum):
    """Domain event categories, translated to wire frames by an ingress adapter."""

    PROGRESS = 'progress'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


@dataclass(frozen=True)
class TaskEvent:
    """Correlated immutable event, with legacy paused/running progress semantics."""

    kind: EventKind
    task_id: str
    sub_id: int
    generation: int
    remaining_distance: float = 0.0
    status: str = 'running'
    detail: str = ''
