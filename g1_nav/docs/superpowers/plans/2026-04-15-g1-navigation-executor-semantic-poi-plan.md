# G1 Navigation Executor Semantic POI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a robot-side WebSocket navigation executor for `g1_nav` that preserves the existing GO2W flat navigation message format, adds `mark_current_poi`, and keeps POI geometry local to the executor so loop-closure updates can reproject POIs without involving the brain.

**Architecture:** First lock the protocol and POI math with pure Python tests. Then add a local YAML-backed POI store plus a testable executor core with fake dependencies. Finally add the ROS 2 / Nav2 / TF adapters and the WebSocket shell, wire the script into `g1_nav`, and verify with targeted pytest plus a package build.

**Tech Stack:** ROS 2 Humble, `rclpy`, `nav2_msgs`, `tf2_ros`, `websockets`, `PyYAML`, Python `pytest`, `ament_cmake`, `ament_cmake_python`, `colcon`

---

## File Map

- Create: `g1_nav/navigation_types.py`
  Shared dataclasses for semantic POIs, poses, transform snapshots, captured poses, and stored POIs.
- Create: `g1_nav/navigation_protocol.py`
  Parse flat incoming JSON commands and build flat outgoing event payloads.
- Create: `g1_nav/poi_store.py`
  Load/save the local YAML POI store, merge semantic directory updates, and reproject POIs after loop closure.
- Create: `g1_nav/navigation_executor_core.py`
  Pure async state machine for `navigate_to`, `abort_navigation`, `update_poi_list`, and `mark_current_poi`.
- Create: `g1_nav/pose_provider.py`
  ROS-facing pose capture adapter that prefers TF `map -> base_link`, falls back to `/lightning/odometry`, and returns fresh snapshots for POI capture.
- Create: `g1_nav/nav2_action_bridge.py`
  ROS-facing Nav2 action adapter that translates `NavigateToPose` callbacks into executor bridge events.
- Create: `g1_nav/navigation_executor.py`
  Installed script that parses CLI args, builds adapters, maintains the WebSocket session, runs heartbeats, and feeds messages into the executor core.
- Create: `test/test_navigation_executor_protocol.py`
  Pure protocol tests for flat JSON parsing and event building.
- Create: `test/test_navigation_executor_poi.py`
  Pure POI store and loop-closure reprojection tests.
- Create: `test/test_navigation_executor_runtime.py`
  Async executor-core tests using fake bridge, fake pose provider, and temporary POI stores.
- Create: `test/test_navigation_executor_packaging.py`
  Static regression tests for CLI parsing, CMake install wiring, and `package.xml` runtime dependencies.
- Modify: `CMakeLists.txt`
  Install the new executor script.
- Modify: `package.xml`
  Add the `python3-websockets` runtime dependency.

### Task 1: Lock the Protocol Contract With Failing Tests

**Files:**
- Create: `test/test_navigation_executor_protocol.py`
- Create: `g1_nav/navigation_types.py`
- Create: `g1_nav/navigation_protocol.py`

- [ ] **Step 1: Write the failing test**

```python
from g1_nav.navigation_protocol import (
    build_mark_poi_success_event,
    build_progress_event,
    parse_command,
)
from g1_nav.navigation_types import PoseRecord, StoredPoi


def test_parse_navigate_to_command_accepts_go2w_flat_schema():
    command = parse_command(
        {
            "action": "navigate_to",
            "request_id": "req_001",
            "sub_id": 7,
            "target_id": "POI_001",
        }
    )

    assert command.action == "navigate_to"
    assert command.request_id == "req_001"
    assert command.sub_id == 7
    assert command.target_id == "POI_001"


def test_parse_mark_current_poi_requires_flat_poi_object():
    command = parse_command(
        {
            "action": "mark_current_poi",
            "request_id": "req_mark",
            "sub_id": 1,
            "poi": {"id": "POI_006", "name": "会议室门口"},
        }
    )

    assert command.action == "mark_current_poi"
    assert command.poi_id == "POI_006"
    assert command.poi_name == "会议室门口"


def test_build_progress_event_rounds_remaining_distance_to_three_decimals():
    payload = build_progress_event("req_001", 7, 1.23456, "running")

    assert payload == {
        "event_type": "on_progress",
        "request_id": "req_001",
        "sub_id": 7,
        "remaining_distance": 1.235,
        "status": "running",
    }


def test_build_mark_poi_success_event_keeps_pose_as_metadata():
    poi = StoredPoi(
        poi_id="POI_006",
        name="会议室门口",
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        slam_session_id="session_a",
    )

    payload = build_mark_poi_success_event("req_mark", 1, poi)

    assert payload["event_type"] == "on_mark_poi_success"
    assert payload["poi"]["id"] == "POI_006"
    assert payload["poi"]["frame_id"] == "map"
    assert payload["poi"]["position"] == {"x": 12.3, "y": 4.5, "z": 0.0}
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_protocol.py
```

Expected: FAIL with `ModuleNotFoundError` because `g1_nav.navigation_protocol` and
`g1_nav.navigation_types` do not exist yet.

- [ ] **Step 3: Write minimal implementation**

Create `g1_nav/navigation_types.py` with the shared pure dataclasses:

```python
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseRecord:
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
    stamp_sec: float

    def orientation(self) -> dict[str, float]:
        half = self.yaw * 0.5
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }


@dataclass(frozen=True)
class StoredPoi:
    poi_id: str
    name: str
    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str
```

Create `g1_nav/navigation_protocol.py` with flat command parsing and payload builders:

```python
from dataclasses import dataclass
from typing import Union

from g1_nav.navigation_types import StoredPoi


@dataclass(frozen=True)
class NavigateToCommand:
    action: str
    request_id: str
    sub_id: int
    target_id: str


@dataclass(frozen=True)
class MarkCurrentPoiCommand:
    action: str
    request_id: str
    sub_id: int
    poi_id: str
    poi_name: str


@dataclass(frozen=True)
class AbortNavigationCommand:
    action: str
    request_id: str


Command = Union[NavigateToCommand, MarkCurrentPoiCommand, AbortNavigationCommand]


def parse_command(payload: dict) -> Command:
    action = str(payload.get("action", "")).strip()
    if action == "navigate_to":
        return NavigateToCommand(
            action="navigate_to",
            request_id=str(payload["request_id"]).strip(),
            sub_id=int(payload["sub_id"]),
            target_id=str(payload["target_id"]).strip(),
        )
    if action == "mark_current_poi":
        poi = payload["poi"]
        return MarkCurrentPoiCommand(
            action="mark_current_poi",
            request_id=str(payload["request_id"]).strip(),
            sub_id=int(payload["sub_id"]),
            poi_id=str(poi["id"]).strip(),
            poi_name=str(poi["name"]).strip(),
        )
    if action == "abort_navigation":
        return AbortNavigationCommand(
            action="abort_navigation",
            request_id=str(payload["request_id"]).strip(),
        )
    raise ValueError(f"unsupported action: {action}")


def build_progress_event(request_id: str, sub_id: int, remaining_distance: float, status: str) -> dict:
    return {
        "event_type": "on_progress",
        "request_id": request_id,
        "sub_id": sub_id,
        "remaining_distance": round(float(remaining_distance), 3),
        "status": status,
    }


def build_mark_poi_success_event(request_id: str, sub_id: int, poi: StoredPoi) -> dict:
    return {
        "event_type": "on_mark_poi_success",
        "request_id": request_id,
        "sub_id": sub_id,
        "poi": {
            "id": poi.poi_id,
            "name": poi.name,
            "frame_id": poi.map_pose.frame_id,
            "position": {
                "x": poi.map_pose.x,
                "y": poi.map_pose.y,
                "z": poi.map_pose.z,
            },
            "yaw": poi.map_pose.yaw,
            "orientation": poi.map_pose.orientation(),
        },
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_protocol.py
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_types.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_protocol.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_protocol.py
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "feat: add navigation executor protocol primitives"
```

### Task 2: Add the Local YAML POI Store and Loop-Closure Reprojection

**Files:**
- Create: `g1_nav/poi_store.py`
- Create: `test/test_navigation_executor_poi.py`
- Modify: `g1_nav/navigation_types.py`

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path

from g1_nav.navigation_types import PoseRecord, Transform2D
from g1_nav.poi_store import LocalPoiStore


def test_apply_semantic_directory_keeps_existing_geometry(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        poi_id="POI_006",
        name="旧名字",
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        slam_session_id="session_a",
    )

    store.apply_semantic_directory([{"id": "POI_006", "name": "会议室门口"}])
    stored = store.get_required("POI_006")

    assert stored.name == "会议室门口"
    assert stored.map_pose.x == 12.3
    assert stored.odom_pose.x == 5.0


def test_reproject_session_updates_map_pose_from_odom_anchor(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        poi_id="POI_006",
        name="会议室门口",
        map_pose=PoseRecord("map", 0.0, 0.0, 0.0, 0.0, 100.0),
        odom_pose=PoseRecord("odom", 1.0, 0.0, 0.0, 0.2, 100.0),
        slam_session_id="session_a",
    )

    changed = store.reproject_session(
        slam_session_id="session_a",
        map_from_odom=Transform2D(x=10.0, y=2.0, yaw=1.0),
        translation_threshold=0.0,
        yaw_threshold=0.0,
    )

    assert [poi.poi_id for poi in changed] == ["POI_006"]
    updated = store.get_required("POI_006")
    assert round(updated.map_pose.x, 3) == 10.54
    assert round(updated.map_pose.y, 3) == 2.841
    assert round(updated.map_pose.yaw, 3) == 1.2
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py
```

Expected: FAIL because `Transform2D` and `LocalPoiStore` do not exist yet.

- [ ] **Step 3: Write minimal implementation**

Extend `g1_nav/navigation_types.py` with the transform type:

```python
@dataclass(frozen=True)
class Transform2D:
    x: float
    y: float
    yaw: float
```

Create `g1_nav/poi_store.py` with YAML-backed storage and reprojection:

```python
from dataclasses import replace
from pathlib import Path
import math
import os

import yaml

from g1_nav.navigation_types import PoseRecord, StoredPoi, Transform2D


def _transform_pose(transform: Transform2D, pose: PoseRecord) -> PoseRecord:
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    x = transform.x + cos_yaw * pose.x - sin_yaw * pose.y
    y = transform.y + sin_yaw * pose.x + cos_yaw * pose.y
    return PoseRecord("map", x, y, pose.z, transform.yaw + pose.yaw, pose.stamp_sec)


class LocalPoiStore:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._pois: dict[str, StoredPoi] = {}

    def load(self) -> None:
        if not self.path.exists():
            return
        payload = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
        self._pois = {}
        for item in payload.get("pois", []):
            self._pois[str(item["id"])] = StoredPoi(
                poi_id=str(item["id"]),
                name=str(item["name"]),
                map_pose=PoseRecord(**item["map_pose"]),
                odom_pose=PoseRecord(**item["odom_pose"]),
                slam_session_id=str(item["slam_session_id"]),
            )

    def get_required(self, poi_id: str) -> StoredPoi:
        return self._pois[poi_id]

    def upsert_marked_poi(
        self,
        poi_id: str,
        name: str,
        map_pose: PoseRecord,
        odom_pose: PoseRecord,
        slam_session_id: str,
    ) -> StoredPoi:
        stored = StoredPoi(poi_id, name, map_pose, odom_pose, slam_session_id)
        self._pois[poi_id] = stored
        self._write()
        return stored

    def apply_semantic_directory(self, semantic_pois: list[dict]) -> None:
        for item in semantic_pois:
            poi_id = str(item["id"])
            if poi_id in self._pois:
                self._pois[poi_id] = replace(self._pois[poi_id], name=str(item["name"]))
        self._write()

    def reproject_session(
        self,
        slam_session_id: str,
        map_from_odom: Transform2D,
        translation_threshold: float,
        yaw_threshold: float,
    ) -> list[StoredPoi]:
        changed: list[StoredPoi] = []
        for poi_id, poi in list(self._pois.items()):
            if poi.slam_session_id != slam_session_id:
                continue
            next_map_pose = _transform_pose(map_from_odom, poi.odom_pose)
            dx = next_map_pose.x - poi.map_pose.x
            dy = next_map_pose.y - poi.map_pose.y
            dyaw = next_map_pose.yaw - poi.map_pose.yaw
            if math.hypot(dx, dy) <= translation_threshold and abs(dyaw) <= yaw_threshold:
                continue
            updated = replace(poi, map_pose=next_map_pose)
            self._pois[poi_id] = updated
            changed.append(updated)
        if changed:
            self._write()
        return changed

    def _write(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.path.with_suffix(self.path.suffix + ".tmp")
        payload = {
            "pois": [
                {
                    "id": poi.poi_id,
                    "name": poi.name,
                    "map_pose": {
                        "frame_id": poi.map_pose.frame_id,
                        "x": poi.map_pose.x,
                        "y": poi.map_pose.y,
                        "z": poi.map_pose.z,
                        "yaw": poi.map_pose.yaw,
                        "stamp_sec": poi.map_pose.stamp_sec,
                    },
                    "odom_pose": {
                        "frame_id": poi.odom_pose.frame_id,
                        "x": poi.odom_pose.x,
                        "y": poi.odom_pose.y,
                        "z": poi.odom_pose.z,
                        "yaw": poi.odom_pose.yaw,
                        "stamp_sec": poi.odom_pose.stamp_sec,
                    },
                    "slam_session_id": poi.slam_session_id,
                }
                for poi in self._pois.values()
            ]
        }
        with tmp_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)
        os.replace(tmp_path, self.path)
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_types.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/poi_store.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "feat: add local POI store and reprojection"
```

### Task 3: Build the Pure Executor Core for Navigate, Abort, and Bridge Events

**Files:**
- Create: `g1_nav/navigation_executor_core.py`
- Create: `test/test_navigation_executor_runtime.py`
- Modify: `g1_nav/navigation_protocol.py`

- [ ] **Step 1: Write the failing test**

```python
import asyncio
from pathlib import Path

import pytest

from g1_nav.navigation_executor_core import ExecutorState, NavigationExecutorCore
from g1_nav.navigation_types import PoseRecord
from g1_nav.poi_store import LocalPoiStore


class FakeBridge:
    def __init__(self):
        self.started = []
        self.canceled = 0

    async def start_navigation(self, request_id, sub_id, poi):
        self.started.append((request_id, sub_id, poi.poi_id))

    async def cancel_current(self):
        self.canceled += 1


class FakePoseProvider:
    async def capture_for_poi(self):
        raise AssertionError("mark_current_poi is not part of this test")


@pytest.mark.asyncio
async def test_navigate_to_starts_bridge_and_progress_event_is_forwarded(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(bridge=bridge, pose_provider=FakePoseProvider(), poi_store=store)

    outbound = await core.handle_message(
        {"action": "navigate_to", "request_id": "req_1", "sub_id": 1, "target_id": "POI_001"}
    )
    forwarded = await core.handle_bridge_event(
        {"kind": "progress", "request_id": "req_1", "sub_id": 1, "remaining_distance": 2.25, "status": "running"}
    )

    assert outbound == []
    assert bridge.started == [("req_1", 1, "POI_001")]
    assert core.state == ExecutorState.STARTING
    assert forwarded == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 1,
            "remaining_distance": 2.25,
            "status": "running",
        }
    ]


@pytest.mark.asyncio
async def test_abort_navigation_emits_paused_progress(tmp_path: Path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    core = NavigationExecutorCore(bridge=bridge, pose_provider=FakePoseProvider(), poi_store=store)

    await core.handle_message({"action": "navigate_to", "request_id": "req_1", "sub_id": 1, "target_id": "POI_001"})
    outbound = await core.handle_message({"action": "abort_navigation", "request_id": "req_1"})

    assert bridge.canceled == 1
    assert outbound == [
        {
            "event_type": "on_progress",
            "request_id": "req_1",
            "sub_id": 1,
            "remaining_distance": 0.0,
            "status": "paused",
        }
    ]
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py
```

Expected: FAIL because `g1_nav.navigation_executor_core` does not exist yet.

- [ ] **Step 3: Write minimal implementation**

Create `g1_nav/navigation_executor_core.py` as a pure async core:

```python
from dataclasses import dataclass
from enum import Enum

from g1_nav.navigation_protocol import (
    build_arrived_event,
    build_error_event,
    build_progress_event,
    parse_command,
)


class ExecutorState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING = "navigating"
    ABORTING = "aborting"
    ERROR = "error"


@dataclass
class ActiveTask:
    request_id: str
    sub_id: int
    target_id: str
    remaining_distance: float = 0.0


class NavigationExecutorCore:
    def __init__(self, bridge, pose_provider, poi_store):
        self.bridge = bridge
        self.pose_provider = pose_provider
        self.poi_store = poi_store
        self.state = ExecutorState.IDLE
        self.active_task = None

    async def handle_message(self, payload: dict) -> list[dict]:
        command = parse_command(payload)
        if command.action == "navigate_to":
            poi = self.poi_store.get_required(command.target_id)
            if self.active_task is not None:
                await self.bridge.cancel_current()
            self.active_task = ActiveTask(command.request_id, command.sub_id, command.target_id)
            self.state = ExecutorState.STARTING
            await self.bridge.start_navigation(command.request_id, command.sub_id, poi)
            return []
        if command.action == "abort_navigation":
            if self.active_task is None or self.active_task.request_id != command.request_id:
                return []
            await self.bridge.cancel_current()
            self.state = ExecutorState.IDLE
            paused = build_progress_event(
                self.active_task.request_id,
                self.active_task.sub_id,
                self.active_task.remaining_distance,
                "paused",
            )
            self.active_task = None
            return [paused]
        raise ValueError(f"unsupported command in core: {command.action}")

    async def handle_bridge_event(self, event: dict) -> list[dict]:
        if self.active_task is None:
            return []
        if event["kind"] == "progress":
            self.state = ExecutorState.NAVIGATING
            self.active_task.remaining_distance = float(event["remaining_distance"])
            return [
                build_progress_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    self.active_task.remaining_distance,
                    event.get("status", "running"),
                )
            ]
        if event["kind"] == "arrived":
            outbound = [build_arrived_event(self.active_task.request_id, self.active_task.sub_id)]
            self.active_task = None
            self.state = ExecutorState.IDLE
            return outbound
        if event["kind"] == "error":
            outbound = [
                build_error_event(
                    self.active_task.request_id,
                    self.active_task.sub_id,
                    event["error_message"],
                )
            ]
            self.active_task = None
            self.state = ExecutorState.ERROR
            return outbound
        return []
```

Extend `g1_nav/navigation_protocol.py` with the missing helpers and abort parsing:

```python
    if action == "abort_navigation":
        return AbortNavigationCommand(
            action="abort_navigation",
            request_id=str(payload["request_id"]).strip(),
        )


def build_arrived_event(request_id: str, sub_id: int) -> dict:
    return {"event_type": "on_arrived", "request_id": request_id, "sub_id": sub_id}


def build_error_event(request_id: str, sub_id: int, error_message: str) -> dict:
    return {
        "event_type": "on_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_protocol.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor_core.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "feat: add navigation executor core state machine"
```

### Task 4: Add `mark_current_poi` Side Tasks and Semantic POI Sync

**Files:**
- Modify: `g1_nav/navigation_executor_core.py`
- Modify: `g1_nav/navigation_protocol.py`
- Modify: `g1_nav/poi_store.py`
- Modify: `test/test_navigation_executor_runtime.py`
- Modify: `test/test_navigation_executor_poi.py`

- [ ] **Step 1: Write the failing test**

```python
import pytest

from g1_nav.navigation_executor_core import NavigationExecutorCore
from g1_nav.navigation_types import CapturedPose, PoseRecord


class FakePoseProvider:
    def __init__(self, captured_pose):
        self.captured_pose = captured_pose
        self.calls = 0

    async def capture_for_poi(self):
        self.calls += 1
        return self.captured_pose


@pytest.mark.asyncio
async def test_mark_current_poi_while_navigating_does_not_cancel_navigation(tmp_path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_001",
        "前台",
        PoseRecord("map", 1.0, 2.0, 0.0, 0.0, 100.0),
        PoseRecord("odom", 1.0, 2.0, 0.0, 0.0, 100.0),
        "session_a",
    )
    bridge = FakeBridge()
    pose_provider = FakePoseProvider(
        CapturedPose(
            map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 101.0),
            odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 101.0),
            slam_session_id="session_a",
        )
    )
    core = NavigationExecutorCore(bridge=bridge, pose_provider=pose_provider, poi_store=store)

    await core.handle_message({"action": "navigate_to", "request_id": "req_nav", "sub_id": 1, "target_id": "POI_001"})
    outbound = await core.handle_message(
        {
            "action": "mark_current_poi",
            "request_id": "req_mark",
            "sub_id": 2,
            "poi": {"id": "POI_006", "name": "会议室门口"},
        }
    )

    assert bridge.canceled == 0
    assert pose_provider.calls == 1
    assert [item["event_type"] for item in outbound] == ["on_mark_poi_ack", "on_mark_poi_success"]
    assert store.get_required("POI_006").name == "会议室门口"


@pytest.mark.asyncio
async def test_mark_current_poi_returns_error_for_stale_pose(tmp_path):
    stale_capture = CapturedPose(
        map_pose=PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 10.0),
        odom_pose=PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 10.0),
        slam_session_id="session_a",
    )
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    bridge = FakeBridge()
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=FakePoseProvider(stale_capture),
        poi_store=store,
        pose_stale_after_sec=0.5,
        now_fn=lambda: 11.0,
    )

    outbound = await core.handle_message(
        {
            "action": "mark_current_poi",
            "request_id": "req_mark",
            "sub_id": 2,
            "poi": {"id": "POI_006", "name": "会议室门口"},
        }
    )

    assert outbound[-1]["event_type"] == "on_mark_poi_error"
    assert outbound[-1]["error_message"] == "当前位置位姿数据过期"


@pytest.mark.asyncio
async def test_update_poi_list_renames_existing_poi_without_touching_geometry(tmp_path):
    store = LocalPoiStore(tmp_path / "poi_store.yaml")
    store.upsert_marked_poi(
        "POI_006",
        "旧名字",
        PoseRecord("map", 12.3, 4.5, 0.0, 1.57, 100.0),
        PoseRecord("odom", 5.0, -1.0, 0.0, 0.3, 100.0),
        "session_a",
    )
    core = NavigationExecutorCore(
        bridge=FakeBridge(),
        pose_provider=FakePoseProvider(
            CapturedPose(
                map_pose=PoseRecord("map", 0.0, 0.0, 0.0, 0.0, 100.0),
                odom_pose=PoseRecord("odom", 0.0, 0.0, 0.0, 0.0, 100.0),
                slam_session_id="session_a",
            )
        ),
        poi_store=store,
    )

    outbound = await core.handle_message(
        {
            "action": "update_poi_list",
            "version": 3,
            "poi_list": [{"id": "POI_006", "name": "会议室门口"}],
        }
    )

    stored = store.get_required("POI_006")
    assert outbound == []
    assert stored.name == "会议室门口"
    assert stored.map_pose.x == 12.3
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py
```

Expected: FAIL because `mark_current_poi`, `CapturedPose`, and stale-pose handling do not exist yet.

- [ ] **Step 3: Write minimal implementation**

Extend `g1_nav/navigation_types.py` with a captured pose type:

```python
@dataclass(frozen=True)
class CapturedPose:
    map_pose: PoseRecord
    odom_pose: PoseRecord
    slam_session_id: str
```

Add flat builders in `g1_nav/navigation_protocol.py`:

```python
@dataclass(frozen=True)
class UpdatePoiListCommand:
    action: str
    version: int
    poi_list: list[dict]


    if action == "update_poi_list":
        return UpdatePoiListCommand(
            action="update_poi_list",
            version=int(payload.get("version", 0)),
            poi_list=list(payload.get("poi_list", [])),
        )


def build_mark_poi_ack(request_id: str, sub_id: int) -> dict:
    return {
        "event_type": "on_mark_poi_ack",
        "request_id": request_id,
        "sub_id": sub_id,
        "status": "accepted",
    }


def build_mark_poi_error(request_id: str, sub_id: int, error_message: str) -> dict:
    return {
        "event_type": "on_mark_poi_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }
```

Update `g1_nav/navigation_executor_core.py` so `mark_current_poi` runs as a side task:

```python
import time

class NavigationExecutorCore:
    def __init__(self, bridge, pose_provider, poi_store, pose_stale_after_sec: float = 0.5, now_fn=None):
        self.bridge = bridge
        self.pose_provider = pose_provider
        self.poi_store = poi_store
        self.pose_stale_after_sec = pose_stale_after_sec
        self.now_fn = now_fn or (lambda: time.time())
        self.state = ExecutorState.IDLE
        self.active_task = None

    async def handle_message(self, payload: dict) -> list[dict]:
        command = parse_command(payload)
        if command.action == "update_poi_list":
            self.poi_store.apply_semantic_directory(command.poi_list)
            return []
        if command.action == "mark_current_poi":
            outbound = [build_mark_poi_ack(command.request_id, command.sub_id)]
            captured = await self.pose_provider.capture_for_poi()
            if self.now_fn() - captured.map_pose.stamp_sec > self.pose_stale_after_sec:
                outbound.append(
                    build_mark_poi_error(command.request_id, command.sub_id, "当前位置位姿数据过期")
                )
                return outbound
            stored = self.poi_store.upsert_marked_poi(
                poi_id=command.poi_id,
                name=command.poi_name,
                map_pose=captured.map_pose,
                odom_pose=captured.odom_pose,
                slam_session_id=captured.slam_session_id,
            )
            outbound.append(build_mark_poi_success_event(command.request_id, command.sub_id, stored))
            return outbound
```

Add semantic-directory merging to `g1_nav/poi_store.py`:

```python
    def apply_semantic_directory(self, semantic_pois: list[dict]) -> None:
        seen_ids = set()
        for item in semantic_pois:
            poi_id = str(item["id"]).strip()
            name = str(item["name"]).strip()
            seen_ids.add(poi_id)
            if poi_id in self._pois:
                self._pois[poi_id] = replace(self._pois[poi_id], name=name)
        self._write()
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_types.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_protocol.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/poi_store.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor_core.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "feat: add mark-current-poi executor flow"
```

### Task 5: Add the ROS Adapters, WebSocket Shell, and Package Wiring

**Files:**
- Create: `g1_nav/pose_provider.py`
- Create: `g1_nav/nav2_action_bridge.py`
- Create: `g1_nav/navigation_executor.py`
- Create: `test/test_navigation_executor_packaging.py`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

- [ ] **Step 1: Write the failing test**

```python
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "g1_nav" / "navigation_executor.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("g1_nav_navigation_executor", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_navigation_executor_cli_parses_server_uri_and_store_path():
    module = _load_module()

    config, ros_args = module._parse_cli_args(
        [
            "--server-uri",
            "ws://10.0.0.5:8100/ws/navigation/executor",
            "--poi-store-file",
            "/tmp/poi_store.yaml",
            "--ros-args",
            "-r",
            "__ns:=/robot",
        ]
    )

    assert config["server_uri"] == "ws://10.0.0.5:8100/ws/navigation/executor"
    assert config["poi_store_file"] == "/tmp/poi_store.yaml"
    assert ros_args == ["--ros-args", "-r", "__ns:=/robot"]


def test_package_xml_declares_python3_websockets():
    root = ET.parse(REPO_ROOT / "package.xml").getroot()
    exec_depends = [node.text for node in root.findall("exec_depend")]

    assert "python3-websockets" in exec_depends


def test_cmake_installs_navigation_executor_script():
    cmake_text = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "g1_nav/navigation_executor.py" in cmake_text
    assert "RENAME navigation_executor.py" in cmake_text
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_packaging.py
```

Expected: FAIL because `g1_nav/navigation_executor.py` does not exist yet and `package.xml`
does not declare `python3-websockets`.

- [ ] **Step 3: Write minimal implementation**

Create `g1_nav/pose_provider.py` with a ROS-facing capture adapter:

```python
import math
import rclpy

from g1_nav.navigation_types import CapturedPose, PoseRecord, Transform2D


class TfPoseProvider:
    def __init__(self, node, tf_buffer, robot_base_frame: str, odom_topic: str, slam_session_id_fn):
        self.node = node
        self.tf_buffer = tf_buffer
        self.robot_base_frame = robot_base_frame
        self.odom_topic = odom_topic
        self.slam_session_id_fn = slam_session_id_fn
        self.latest_odom_pose = None

    async def capture_for_poi(self) -> CapturedPose:
        map_pose = self._lookup_map_pose()
        odom_pose = self._lookup_odom_pose()
        return CapturedPose(
            map_pose=map_pose,
            odom_pose=odom_pose,
            slam_session_id=self.slam_session_id_fn(),
        )

    def current_slam_session_id(self) -> str:
        return self.slam_session_id_fn()

    def current_map_from_odom(self) -> Transform2D:
        transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return Transform2D(
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            yaw=yaw,
        )
```

Create `g1_nav/nav2_action_bridge.py` with the same event vocabulary the core already expects:

```python
import asyncio

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from g1_nav.navigation_types import StoredPoi


class Nav2ActionBridge:
    def __init__(self, node, action_name: str, server_timeout: float, event_callback):
        self.node = node
        self.action_name = action_name
        self.server_timeout = server_timeout
        self.event_callback = event_callback
        self.client = ActionClient(node, NavigateToPose, action_name)
        self._current_goal_handle = None

    async def start_navigation(self, request_id: str, sub_id: int, poi: StoredPoi) -> None:
        if not self.client.wait_for_server(timeout_sec=self.server_timeout):
            await self.event_callback(
                {
                    "kind": "error",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "error_message": f"Nav2 action server is not available within {self.server_timeout:.1f}s",
                }
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = poi.map_pose.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = poi.map_pose.x
        goal.pose.pose.position.y = poi.map_pose.y
        goal.pose.pose.position.z = poi.map_pose.z
        orientation = poi.map_pose.orientation()
        goal.pose.pose.orientation.x = orientation["x"]
        goal.pose.pose.orientation.y = orientation["y"]
        goal.pose.pose.orientation.z = orientation["z"]
        goal.pose.pose.orientation.w = orientation["w"]

        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: asyncio.create_task(
                self.event_callback(
                    {
                        "kind": "progress",
                        "request_id": request_id,
                        "sub_id": sub_id,
                        "remaining_distance": float(feedback.feedback.distance_remaining),
                        "status": "running",
                    }
                )
            ),
        )
        goal_handle = await asyncio.wrap_future(send_future)
        self._current_goal_handle = goal_handle
        result = await asyncio.wrap_future(goal_handle.get_result_async())
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            await self.event_callback({"kind": "arrived", "request_id": request_id, "sub_id": sub_id})
        elif result.status == GoalStatus.STATUS_CANCELED:
            await self.event_callback({"kind": "progress", "request_id": request_id, "sub_id": sub_id, "remaining_distance": 0.0, "status": "paused"})
        else:
            await self.event_callback(
                {
                    "kind": "error",
                    "request_id": request_id,
                    "sub_id": sub_id,
                    "error_message": f"Nav2 finished with status {result.status}",
                }
            )

    async def cancel_current(self) -> None:
        if self._current_goal_handle is not None:
            await asyncio.wrap_future(self._current_goal_handle.cancel_goal_async())
```

Create `g1_nav/navigation_executor.py` as the installed script:

```python
#!/usr/bin/env python3
import asyncio
import json
import os


DEFAULT_SERVER_URI = "ws://192.168.0.131:8100/ws/navigation/executor"
DEFAULT_POI_STORE_FILE = os.path.expanduser("~/.ros/g1_nav/poi_store.yaml")
DEFAULT_LOOP_CLOSURE_POLL_SEC = 1.0


def _parse_cli_args(raw_args):
    config = {
        "server_uri": DEFAULT_SERVER_URI,
        "poi_store_file": DEFAULT_POI_STORE_FILE,
    }
    ros_args = []
    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == "--server-uri" and i + 1 < len(raw_args):
            config["server_uri"] = raw_args[i + 1]
            i += 2
            continue
        if arg == "--poi-store-file" and i + 1 < len(raw_args):
            config["poi_store_file"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        ros_args.append(arg)
        i += 1
    return config, ros_args


async def _send_heartbeat(websocket, interval_sec: float):
    while True:
        await asyncio.sleep(interval_sec)
        await websocket.send(json.dumps({"type": "ping"}, ensure_ascii=False))


async def _watch_loop_closure(pose_provider, poi_store, poll_sec: float):
    last_transform = None
    while True:
        await asyncio.sleep(poll_sec)
        current_transform = pose_provider.current_map_from_odom()
        if current_transform == last_transform:
            continue
        poi_store.reproject_session(
            slam_session_id=pose_provider.current_slam_session_id(),
            map_from_odom=current_transform,
            translation_threshold=0.05,
            yaw_threshold=0.03,
        )
        last_transform = current_transform


async def main_async(raw_args):
    import rclpy
    import websockets
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    from g1_nav.navigation_executor_core import NavigationExecutorCore
    from g1_nav.nav2_action_bridge import Nav2ActionBridge
    from g1_nav.poi_store import LocalPoiStore
    from g1_nav.pose_provider import TfPoseProvider

    config, ros_args = _parse_cli_args(raw_args)
    rclpy.init(args=ros_args)
    node = Node("navigation_executor")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    poi_store = LocalPoiStore(config["poi_store_file"])
    poi_store.load()

    outbound_queue: asyncio.Queue = asyncio.Queue()

    async def on_bridge_event(event: dict):
        for payload in await core.handle_bridge_event(event):
            await outbound_queue.put(payload)

    bridge = Nav2ActionBridge(node=node, action_name="/navigate_to_pose", server_timeout=10.0, event_callback=on_bridge_event)
    pose_provider = TfPoseProvider(node=node, tf_buffer=tf_buffer, robot_base_frame="base_link", odom_topic="/lightning/odometry", slam_session_id_fn=lambda: "live_session")
    core = NavigationExecutorCore(bridge=bridge, pose_provider=pose_provider, poi_store=poi_store)

    try:
        async with websockets.connect(config["server_uri"], ping_interval=None) as websocket:
            heartbeat_task = asyncio.create_task(_send_heartbeat(websocket, 30.0))
            loop_closure_task = asyncio.create_task(_watch_loop_closure(pose_provider, poi_store, DEFAULT_LOOP_CLOSURE_POLL_SEC))
            try:
                async for raw_message in websocket:
                    payload = json.loads(raw_message)
                    if payload.get("type") == "pong":
                        continue
                    for outbound in await core.handle_message(payload):
                        await websocket.send(json.dumps(outbound, ensure_ascii=False))
                    while not outbound_queue.empty():
                        await websocket.send(json.dumps(await outbound_queue.get(), ensure_ascii=False))
            finally:
                heartbeat_task.cancel()
                loop_closure_task.cancel()
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()
```

Modify `package.xml` to add the missing runtime dependency:

```xml
<exec_depend>python3-websockets</exec_depend>
```

Modify `CMakeLists.txt` to install the script:

```cmake
install(PROGRAMS
  g1_nav/navigation_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigation_executor.py
)
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_packaging.py
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/pose_provider.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/nav2_action_bridge.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_packaging.py \
  /home/unitree/ros2_ws/src/g1_nav/CMakeLists.txt \
  /home/unitree/ros2_ws/src/g1_nav/package.xml
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "feat: wire g1 navigation executor runtime"
```

### Task 6: Verify the Full Plan Scope

**Files:**
- Re-run the new tests and package build only

- [ ] **Step 1: Run the pure executor regression suite**

Run:

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_nav:$PYTHONPATH \
python3 -m pytest -q \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_protocol.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_packaging.py
```

Expected: PASS

- [ ] **Step 2: Build `g1_nav` with an isolated build/install base**

Run:

```bash
colcon --log-base /tmp/g1_nav_log build \
  --packages-select g1_nav \
  --build-base /tmp/g1_nav_build \
  --install-base /tmp/g1_nav_install \
  --event-handlers console_direct+
```

Expected: `build` exits with code `0`.

- [ ] **Step 3: Run the package test targets from the isolated build**

Run:

```bash
colcon --log-base /tmp/g1_nav_log test \
  --packages-select g1_nav \
  --build-base /tmp/g1_nav_build \
  --install-base /tmp/g1_nav_install \
  --event-handlers console_direct+
colcon --log-base /tmp/g1_nav_log test-result --verbose
```

Expected:

- the new pytest files are discovered and pass
- existing `g1_nav` tests stay green
- `test-result --verbose` reports no failing test cases

- [ ] **Step 4: Smoke-test the installed executor CLI help**

Run:

```bash
source /tmp/g1_nav_install/setup.bash
ros2 run g1_nav navigation_executor.py --help
```

Expected: the help text prints `--server-uri` and `--poi-store-file`.

- [ ] **Step 5: Commit verification-only follow-ups if verification changed executor files**

```bash
git -C /home/unitree/ros2_ws/src/g1_nav status --short
```

Expected: no unexpected uncommitted changes remain after the verification run. If verification
forced small fixes in the executor files from this plan, commit them with:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_types.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_protocol.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/poi_store.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor_core.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/pose_provider.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/nav2_action_bridge.py \
  /home/unitree/ros2_ws/src/g1_nav/g1_nav/navigation_executor.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_protocol.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_poi.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_runtime.py \
  /home/unitree/ros2_ws/src/g1_nav/test/test_navigation_executor_packaging.py \
  /home/unitree/ros2_ws/src/g1_nav/CMakeLists.txt \
  /home/unitree/ros2_ws/src/g1_nav/package.xml
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "test: fix navigation executor verification regressions"
```
