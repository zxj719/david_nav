"""Synchronous Dock worker ports, independent of ROS and transport implementations."""

from dataclasses import dataclass
from typing import Protocol

PlanarPose = tuple[float, float, float]


@dataclass(frozen=True)
class DockActionResult:
    """Settled action verdict and whether an explicit task cancel caused the return.

    A false result is not permission to abandon motion. The implementing bridge
    must retain ownership until cleanup is complete, even when a deadline expires.
    External CANCELED, unlike an explicit task cancel, retains the old failed
    stage semantics and may still lead to measured stage-two acceptance.
    """

    succeeded: bool
    canceled: bool = False


class DockMotionPort(Protocol):
    """Blocking worker view of the sole shared motion backend, never a second owner."""

    def navigate(
        self, pose: PlanarPose, label: str, timeout_sec: float, extra_distance: float,
    ) -> DockActionResult:
        """Run one navigation stage with the frozen action-status audit lines.

        Args:
            pose: Navigation-base x/y/yaw in the configured global frame.
            label: Frozen stage label for logging.
            timeout_sec: Stage result deadline, separate from admission/cleanup deadlines.
            extra_distance: Add this distance to forwarded stage feedback.

        Returns:
            Stage success and explicit-cancel state, after resolving motion ownership.
        """
        ...

    def spin(
        self, yaw: float, label: str, time_allowance_sec: float, timeout_sec: float,
    ) -> DockActionResult:
        """Run one relative rotation through the same backend as navigation.

        Args:
            yaw: Signed relative yaw in radians.
            label: Frozen correction-attempt label.
            time_allowance_sec: Server-side execution allowance.
            timeout_sec: Local result wait, separate from cleanup deadline.

        Returns:
            Rotation success and explicit-cancel state, after resolving ownership.
        """
        ...


class DockLogger(Protocol):
    """Injected thread-safe logger using the existing ROS logger method names."""

    def info(self, text: str) -> None:
        """Record an informational frozen line.

        Args:
            text: Complete log line including the dock prefix.
        """
        ...

    def warn(self, text: str) -> None:
        """Record a warning frozen line.

        Args:
            text: Complete log line including the dock prefix.
        """
        ...

    def error(self, text: str) -> None:
        """Record an error frozen line.

        Args:
            text: Complete log line including the dock prefix.
        """
        ...
