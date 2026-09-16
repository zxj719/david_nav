"""Full-body static-map admission, deliberately separate from ordinary POI snapping."""

from typing import Sequence

from nav_tasks.domain.footprint import check_footprint, FootprintCheck, OccupancyGridView
from nav_tasks.skills.docking.ports import PlanarPose


def check_approach_footprint(
    grid: OccupancyGridView | None, global_frame: str,
    footprint: Sequence[tuple[float, float]], pose: PlanarPose, occupied_threshold: int,
) -> FootprintCheck:
    """Reject missing/wrong-frame maps before evaluating the complete placed footprint.

    Args:
        grid: Immutable latest static-map snapshot, or None before receipt.
        global_frame: Explicit frame used for the goal and approach pose.
        footprint: Navigation-base polygon, not a center point or clearance radius.
        pose: Approach x/y/yaw in the configured global frame.
        occupied_threshold: Inclusive occupied-cell threshold; unknown cells remain allowed.

    Returns:
        Original clear/rejection reason and first occupied cell when present.
    """
    if grid is None:
        return FootprintCheck(False, 'static_map_not_received')
    if grid.frame_id != global_frame:
        return FootprintCheck(
            False, 'static_map_frame_mismatch:%s!=%s' % (grid.frame_id, global_frame))
    return check_footprint(grid, footprint, pose, occupied_threshold)
