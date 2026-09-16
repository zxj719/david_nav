"""Read the manual named-poses JSON source without modifying its contents."""

import json
from pathlib import Path
from typing import Callable

from nav_tasks.core.ports import PoiRepository
from nav_tasks.domain.types import PoseRecord, StoredPoi


def import_named_poses(
    repository: PoiRepository, path: Path, *, map_frame: str, odom_frame: str,
    log: Callable[[str], None] | None = None,
) -> int:
    """Import old named-poses shapes and prune only the named_poses session.

    Coordinates are copied verbatim into both pose representations, exactly as
    before; no reinterpretation of manually maintained frame semantics occurs.
    Writes remain per-record and pruning follows successful iteration, not a new
    whole-import transaction. Malformed records retain legacy exceptions.

    Args:
        repository: Writable executor-owned POI repository.
        path: Read-only manual JSON source.
        map_frame: Fallback frame for entries without frame_id.
        odom_frame: Frame label for the legacy copied odometry representation.
        log: Optional unprefixed runtime log sink for stale-ID pruning.

    Returns:
        Number of imported mapping entries, including repeated IDs.
    """
    with Path(path).open('r', encoding='utf-8') as handle:
        payload = json.load(handle)
    items = (payload.get('poses') or payload.get('named_poses') or []) if (
        isinstance(payload, dict)) else payload
    count = 0
    imported = set()
    for entry in items:
        if not isinstance(entry, dict):
            continue
        position = entry.get('position', {}) or {}
        poi_id = str(entry['id'])
        name = str(entry.get('name', poi_id))
        frame = str(entry.get('frame_id', map_frame))
        x, y, z = (float(position.get(axis, 0.0)) for axis in ('x', 'y', 'z'))
        yaw = float(entry.get('yaw', 0.0))
        repository.upsert(StoredPoi(
            poi_id, name, PoseRecord(frame, x, y, z, yaw, 0.0),
            PoseRecord(odom_frame, x, y, z, yaw, 0.0), 'named_poses'))
        imported.add(poi_id)
        count += 1
    removed = repository.prune_session('named_poses', imported)
    if removed and log is not None:
        log(f'pruned stale named_poses POIs: {sorted(removed)}')
    return count
