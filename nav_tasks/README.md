# nav_tasks

Transport-independent navigation domain and pure skills (Python 3.10+).
This first P6.1 migration does **not** start a task manager or switch a runtime
entry point. Core ports, registry, ownership, retry and supervision follow next.

```python
from nav_tasks.api import PoseRecord
from nav_tasks.skills.docking.settle import SettleSpec, is_settled

pose = PoseRecord('example_map', 1.0, 2.0, 0.0, 0.0, 10.0)
assert pose.orientation()['w'] == 1.0
assert is_settled([(0.0, 0.0), (0.4, 0.0)], SettleSpec())
```

`domain/types.py` owns PoseRecord, StoredPoi, Transform2D and CapturedPose.
`domain/grid.py` retains the legacy axis-aligned grid conversion and occupancy
rules; this relocation does not add origin rotation or new input validation.
`skills/docking/{settle,finish}.py` retain the complete existing pure algorithms.
No ROS, WebSocket, YAML or filesystem implementation is imported by these layers.

The default settling threshold remains **0.05 rad/s**, continuously satisfied for
0.4 s with a 0.05 s polling interval. `DockTolerance` still has the old helper
defaults (xy 0.10 m, yaw 0.087 rad, two retries). Those are **not** David's runtime
tuning: the robot/mode configuration explicitly supplies its selected values.
No physical stop or accuracy claim follows from the pure helper tests.

The five corresponding legacy modules in `david_nav` remain compatibility
exports. Old and new imports refer to the same classes/functions, preserving
type identity rather than creating parallel implementations. Configuration
schema is currently empty because pure functions consume explicit arguments;
profile-key ownership moves with the later runtime integration.

Tests cover pure behavior, isolated imports, architecture boundaries, Google
docstrings and formatting. Existing `david_nav` tests and G1–G16 remain unchanged
and exercise these implementations through compatibility imports.

See [migration.md](docs/migration.md) for ownership and unfinished work.
