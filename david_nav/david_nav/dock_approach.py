"""Compatibility exports for the shared static-map footprint geometry."""

from nav_tasks.domain.footprint import (
    check_footprint, FootprintCheck, OccupancyGridView, parse_footprint,
)

__all__ = ['check_footprint', 'FootprintCheck', 'OccupancyGridView', 'parse_footprint']
