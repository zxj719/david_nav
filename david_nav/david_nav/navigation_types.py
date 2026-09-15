"""Compatibility exports for nav_tasks.domain.types; new logic belongs there."""

from nav_tasks.domain.types import (
    CapturedPose, PoseRecord, StoredPoi, Transform2D,
)

__all__ = [
    'CapturedPose',
    'PoseRecord',
    'StoredPoi',
    'Transform2D',
]
