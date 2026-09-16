"""Backend, localization, persistence and resolution contracts for task skills."""

from typing import Callable, Iterable, Protocol

from nav_tasks.domain.tasks import NavGoal, NavOutcome, ResolvedGoal, SpinGoal
from nav_tasks.domain.types import CapturedPose, PoseRecord, StoredPoi, Transform2D

FeedbackCallback = Callable[[float], None]


class NavigationBackend(Protocol):
    """Execute one attempt only; task skills own retry policy."""

    async def navigate(self, goal: NavGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """Wait for a terminal outcome while reporting loop-thread feedback.

        Args:
            goal: Target pose and request context for logging.
            on_feedback: Remaining distance callback; never called from a ROS thread.

        Returns:
            Success, external cancellation, or a readable failure.

        Raises:
            asyncio.CancelledError: Propagated after arranging cancellation of any
                in-flight goal, including a goal handle that has not arrived yet.
        """
        ...

    async def cancel(self) -> None:
        """Cancel the owned goal, including pending admission; no goal is a no-op.

        Completion means that it is safe to admit another goal. Implementations
        must not acknowledge cleanup before a late handle has been dealt with.
        """
        ...


class SpinBackend(Protocol):
    """One relative rotation sharing the navigation backend's motion ownership."""

    async def spin(self, goal: SpinGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """Wait for one Spin outcome without applying docking retry or settling policy.

        Args:
            goal: Signed relative yaw and server-side time allowance.
            on_feedback: Angular distance traveled in radians on the owning loop.

        Returns:
            Success, external cancellation, or one-attempt failure.

        Raises:
            asyncio.CancelledError: Caller cancellation propagates after arranging cleanup.
        """
        ...

    async def cancel(self) -> None:
        """Settle the shared motion slot, including any late accepted goal handle."""
        ...


class PoseSource(Protocol):
    """Localization observations without ownership of the localization stack."""

    def current_map_pose(self) -> PoseRecord:
        """Read the current global pose.

        Returns:
            Latest global-frame pose.

        Raises:
            PoseUnavailableError: Required localization data are unavailable.
        """
        ...

    async def capture_for_poi(self) -> CapturedPose:
        """Capture both pose representations and their session identity.

        Returns:
            Coherent POI capture.

        Raises:
            PoseUnavailableError: Either required pose cannot be obtained.
        """
        ...

    def current_map_from_odom(self) -> Transform2D:
        """Read the global-from-odometry planar transform.

        Returns:
            Current transform for POI reprojection.
        """
        ...

    def current_slam_session_id(self) -> str:
        """Read the localization session identity.

        Returns:
            Session identifier supplied by the localization adapter.
        """
        ...


class PoiRepository(Protocol):
    """POI storage; concrete writers must use atomic file replacement."""

    def load(self) -> None:
        """Load repository data without modifying external source maps."""
        ...

    def get(self, poi_id: str) -> StoredPoi | None:
        """Look up a POI.

        Args:
            poi_id: Stable POI identifier.

        Returns:
            Stored value or None when absent.
        """
        ...

    def upsert(self, poi: StoredPoi) -> StoredPoi:
        """Atomically persist a POI.

        Args:
            poi: Value to persist.

        Returns:
            Persisted value.
        """
        ...

    def rename_many(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """Apply directory names without creating unmeasured POIs.

        Args:
            id_to_name: Pairs of POI identifier and new display name.
        """
        ...

    def prune_session(self, session_id: str, keep_ids: set[str]) -> list[str]:
        """Remove obsolete POIs only within the specified session.

        Args:
            session_id: Session whose directory is authoritative.
            keep_ids: POI identifiers that must remain.

        Returns:
            Removed identifiers.
        """
        ...

    def reproject_session(
        self, session_id: str, map_from_odom: Transform2D,
        translation_threshold_m: float, yaw_threshold_rad: float,
    ) -> list[StoredPoi]:
        """Reproject stored odometry anchors after a localization correction.

        Args:
            session_id: Session to update.
            map_from_odom: Corrected global-from-odometry transform.
            translation_threshold_m: Minimum relevant translational change.
            yaw_threshold_rad: Minimum relevant angular change.

        Returns:
            Updated POIs.
        """
        ...


class GoalResolver(Protocol):
    """Resolve an anchor or a free-space substitute without moving the robot."""

    def resolve(self, poi: StoredPoi) -> ResolvedGoal | None:
        """Resolve the first navigable target.

        Args:
            poi: Requested anchor.

        Returns:
            Resolution or None when no target is admissible.
        """
        ...

    def is_admissible(self, poi: StoredPoi) -> bool:
        """Check whether the anchor is now usable after the snap segment.

        Args:
            poi: Original requested anchor.

        Returns:
            Whether navigation may proceed to the anchor.
        """
        ...


class Clock(Protocol):
    """Injectable monotonic clock; tests need no wall-clock sleeps."""

    def now(self) -> float:
        """Read monotonic time.

        Returns:
            Time in seconds.
        """
        ...

    async def sleep(self, seconds: float) -> None:
        """Yield for an interval, propagating asyncio cancellation.

        Args:
            seconds: Nonnegative duration in seconds.
        """
        ...
