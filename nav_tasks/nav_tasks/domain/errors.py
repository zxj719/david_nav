"""Typed domain and task-admission failures without transport dependencies."""


class NavTaskError(Exception):
    """Base class for expected navigation task failures."""


class PoseUnavailableError(NavTaskError):
    """A required pose or transform cannot currently be obtained."""


class PoiNotFoundError(NavTaskError):
    """The requested POI does not exist in the repository."""


class ConfigError(NavTaskError):
    """Task configuration is invalid before execution begins."""


class BusyError(NavTaskError):
    """An existing task owns motion and the new request cannot preempt it."""


class ClosedError(NavTaskError):
    """Task admission or background work has already been closed."""


class UnknownActionError(NavTaskError):
    """No decoder and handler are registered for the requested action."""
