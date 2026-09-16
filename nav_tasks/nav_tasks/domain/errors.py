"""Typed domain and task-admission failures without transport dependencies."""

from typing import Awaitable, Callable


class NavTaskError(Exception):
    """Base class for expected navigation task failures."""


class PoseUnavailableError(NavTaskError):
    """A required pose or transform cannot currently be obtained."""


class PoiNotFoundError(NavTaskError):
    """The requested POI does not exist in the repository."""


class ConfigError(NavTaskError):
    """Task configuration is invalid before execution begins."""


class RuntimeAssemblyError(NavTaskError):
    """Construction failed with resources still owned by a retryable cleanup handle.

    Attributes:
        cleanup: 在原事件循环重试清理的异步回调；成功前资源仍归原宿主。
    """

    def __init__(
        self, cause: BaseException, cleanup: Callable[[], Awaitable[None]],
    ) -> None:
        """Retain cleanup until it succeeds, before destroying the external ROS node.

        Args:
            cause: Original construction failure, also retained as the exception cause.
            cleanup: Retry failed destruction operations on the original event loop.
        """
        super().__init__(f'runtime construction cleanup is incomplete: {cause}')
        self.cleanup = cleanup


class BusyError(NavTaskError):
    """An existing task owns motion and the new request cannot preempt it."""


class ClosedError(NavTaskError):
    """Task admission or background work has already been closed."""


class UnknownActionError(NavTaskError):
    """No decoder and handler are registered for the requested action."""
