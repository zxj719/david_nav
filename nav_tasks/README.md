# nav_tasks

The [Dock ROS ingress](docs/dock-ingress.md) now routes topic/action requests
through the shared manager, preserving busy rejection, cancellation and full
feedback. Its public owner requires the multithreaded host shutdown sequence;
the nav_executor composition root now provides that host. Retained david_nav entries
forward to it; deployed production files are unchanged.

The [Dock parameter bridge](docs/dock-parameters.md) accepts existing rendered
ROS parameters and preserves their declaration/override contract. It does not
own an executable; the outer host requires explicit complete policy.

Transport-independent navigation domain and pure skills (Python 3.10+).
The P6.1 core now provides a fake-testable `TaskManager`/`TaskApi`, ports, registry,
ownership, retry and supervision. It does **not** start a task manager process or
own a runtime entry point; nav_executor composes it and owns the retained CLI shims.
Sequential POI navigation, pure snap search and administrative POI handlers are
implemented against fake-testable ports, with no subscriptions or storage I/O.
The shared [Nav2 backend](docs/nav2-backend.md) implements one navigation or opt-in
Spin attempt on the same motion slot and owns late-handle cleanup; the retained
entry points now reach it through nav_executor.
`api.PoiApi` exposes mark/rename operations separately from exclusive motion
ownership. The new `nav_executor` command adapter uses only this public module;
its fake-action integration tests exercise the new chain's G1–G15 wire scenarios.
Public snapshots include skill-supplied target/phase labels, fenced against stale
callbacks. Optional immutable retry observations drive the frozen RetryAudit
logger outside pure policy. New-chain tests cover G16 STATUS and G8/G15 audit
text. The public [runtime factory](docs/runtime.md) now assembles ordinary POI
services with retryable cleanup. nav_executor now supplies CLI/help, WS and an
outer ROS lifetime for ordinary POIs. The public factory now also assembles Dock
from explicit DockRuntimeConfig, with independent static-map/navigation-frame
observations and routing under the same task manager. Explicit rendered Dock
files now bind through the CLI/public parameter bridge and ROS ingress, including
the retained Dock shim and precise launch.

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
The blocking [Dock workflow](docs/dock-workflow.md) now sequences approach
admission, two navigation stages and settled-pose judgement behind worker ports.
Its logs/results match legacy methods. DockWorker now owns and joins a real thread,
bridges all actions to the same Nav2Client, preserves admission/result deadlines
and fences canceled verdicts. Explicit factory configuration, outer ROS ingress
and host integration are available in nav_executor. The
blocking policy must not run on the asyncio or ROS callback thread.
`domain/footprint.py` owns the unchanged full-body static-map checker, also used
through the `david_nav.dock_approach` compatibility exports.
`core/handlers/navigate_to.py` sequences snap/anchor goals with per-segment retries;
`core/handlers/poi_commands.py` marks/renames POIs without taking motion ownership.
`core/goal_resolution.py` preserves the legacy snap search under differential tests.
No ROS, WebSocket, YAML or filesystem implementation is imported by these layers.
Concrete ROS adapters are kept outside the pure layers and import ROS lazily.
The [site data boundary](docs/site-data.md) now supplies an actual ROS-free YAML
PoiRepository and read-only named-poses importer. It is physically hosted under
`nav_tasks.nav_site_data` per PLAN, with byte-level old/new persistence tests;
legacy entry points forward to the shared outer runtime.
The [observation adapters](docs/observation-adapters.md) now supply configured
TF/odometry and static-grid/costmap ports plus supervised loop-closure polling.
Their actual code composes with storage and the new task chain under fake ROS I/O;
they do not start localization, spin an executor or switch the runtime entry point.

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
