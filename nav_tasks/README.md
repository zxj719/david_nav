# nav_tasks

Transport-independent navigation domain and pure skills (Python 3.10+).
The P6.1 core now provides a fake-testable `TaskManager`/`TaskApi`, ports, registry,
ownership, retry and supervision. It does **not** start a task manager process or
switch a runtime entry point; the old core remains active pending integration.
Sequential POI navigation, pure snap search and administrative POI handlers are
implemented against fake-testable ports, with no subscriptions or storage I/O.
The shared [Nav2 backend](docs/nav2-backend.md) implements one navigation attempt
and owns late-handle cleanup; it is not yet connected to legacy entry points.
`api.PoiApi` exposes mark/rename operations separately from exclusive motion
ownership. The new `nav_executor` command adapter uses only this public module;
its fake-action integration tests exercise the new chain's G1–G15 wire scenarios.
Public snapshots include skill-supplied target/phase labels, fenced against stale
callbacks. Optional immutable retry observations drive the frozen RetryAudit
logger outside pure policy. New-chain tests cover G16 STATUS and G8/G15 audit
text; actual runtime assembly and G16 CLI/help integration remain pending.

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
`core/handlers/navigate_to.py` sequences snap/anchor goals with per-segment retries;
`core/handlers/poi_commands.py` marks/renames POIs without taking motion ownership.
`core/goal_resolution.py` preserves the legacy snap search under differential tests.
No ROS, WebSocket, YAML or filesystem implementation is imported by these layers.
Concrete ROS adapters are kept outside the pure layers and import ROS lazily.

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

See [architecture.md](docs/architecture.md) for cancellation/preemption contracts
and [migration.md](docs/migration.md) for package ownership and unfinished work.
