"""Explicit runtime values, independent of CLI, profiles and ROS parameter APIs."""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RuntimeConfig:
    """Task-service assembly configuration supplied by a robot-aware outer root.

    Robot-specific frames, topics, action names and storage paths are required;
    this type does not guess a robot profile or modify localization wiring.
    Optional numeric defaults retain the old executor's policies. Dock IDs are
    carried explicitly so an incomplete assembly cannot silently navigate them
    with the ordinary POI skill before Dock integration is available.
    """

    poi_store_file: Path
    robot_base_frame: str
    odom_topic: str
    map_frame: str
    odom_frame: str
    grid_map_topic: str
    global_costmap_topic: str
    action_name: str
    slam_session_id: str
    pose_file: Path | None = None
    source_poi_file: Path | None = None
    dock_poi_ids: tuple[str, ...] = ()
    server_timeout_sec: float = 10.0
    cancel_timeout_sec: float = 5.0
    nav_retry_count: int = 1
    nav_retry_delay_sec: float = 1.0
    nav_goal_snap_radius: float = 0.5
    nav_goal_fallback_snap_radius: float = 1.0
    nav_goal_clearance_radius: float = 0.1
    nav_goal_max_snap_radius: float = 3.0
    nav_goal_snap_step: float = 0.25
    loop_closure_poll_sec: float = 1.0
    loop_closure_translation_m: float = 0.05
    loop_closure_yaw_rad: float = 0.03
    progress_min_interval_sec: float = 1.0
    pose_stale_after_sec: float = 0.5
