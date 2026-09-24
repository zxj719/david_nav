# nav_tasks

Transport-independent navigation domain, task ownership and pure skills.
`TaskManager` is the sole owner of navigation/Dock work; `nav_executor` provides
the process, ROS lifetime and ingress composition root.

The package exposes its supported surface through `nav_tasks.api`. Pure modules
under `domain`, `core` and `skills` do not import ROS, WebSockets, YAML or storage
implementations. ROS and persistence live behind adapters.

```python
from nav_tasks.api import PoseRecord
from nav_tasks.skills.docking.settle import SettleSpec, is_settled

pose = PoseRecord('example_map', 1.0, 2.0, 0.0, 0.0, 10.0)
assert pose.orientation()['w'] == 1.0
assert is_settled([(0.0, 0.0), (0.4, 0.0)], SettleSpec())
```

The shared Nav2 backend serializes NavigateToPose and Spin on one motion slot and
owns late-handle cancellation. Dock uses the same manager and backend; blocking
workflow code runs in an owned worker, never on the asyncio or ROS callback thread.
Cleanup must succeed before task ownership is released.

Robot values—including frames, topics, Dock tolerances and footprint—arrive as
explicit rendered configuration. The package contains no alternate startup path
or robot-specific default pipeline.

See [architecture](docs/architecture.md), [runtime](docs/runtime.md),
[Dock workflow](docs/dock-workflow.md), and [site data](docs/site-data.md).
