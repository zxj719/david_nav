# 2026-04-15 G1 Navigation Executor Semantic POI Design

## Summary

This design adds a robot-side WebSocket navigation executor to `g1_nav` that
matches the existing `go2w_real` message style for navigation while extending
it with "mark current POI" support.

The core architectural decision is:

- the brain/server owns semantic intent
- the executor owns geometric truth

That means the brain can ask the robot to navigate to `POI_001` or mark the
current place as `POI_006 / 会议室门口`, but it does not need to own or persist
the live map coordinates of those POIs. The executor keeps the local geometric
POI store, resolves `target_id` into live poses, and updates those poses when
SLAM loop closure shifts the global map.

This design intentionally stays close to the existing
`go2w_real/scripts/navigation_executor.py` chain so the old sender-side message
format remains familiar and compatible:

- flat JSON messages
- `navigate_to`, `abort_navigation`
- `on_progress`, `on_arrived`, `on_error`
- WebSocket client on the robot
- WebSocket server on the cloud/brain side

## Problem

`g1_nav` currently provides Nav2 bringup, exploration, and robot-side
navigation execution, but it does not expose a robot-side WebSocket executor
that an external brain can drive.

The new system must support two classes of behavior:

1. classic external navigation dispatch
2. live POI creation from the robot's current position

The POI requirement introduces a second problem beyond plain navigation:

- POIs may be created while the robot is moving
- SLAM loop closure may later move the global `map` frame
- a POI recorded once in `map` coordinates can become stale

If the server owns POI geometry, loop-closure correction becomes a distributed
consistency problem. If the executor owns POI geometry locally, the executor
can keep POIs aligned with the active SLAM session and expose only semantic IDs
to the brain.

## Goals

- Add a robot-side WebSocket executor to `g1_nav`.
- Keep the old navigation message format compatible with the existing
  `go2w_real` executor style.
- Allow the brain to stay at the semantic layer instead of owning map geometry.
- Support `mark_current_poi` without interrupting ongoing navigation.
- Allow POIs recorded during motion to remain locally correct after SLAM loop
  closure updates the global map.
- Keep the protocol and code structure testable in isolation from the full
  navigation stack.

## Non-Goals

- Making the cloud/server the source of truth for POI geometry.
- Designing a general-purpose map synchronization protocol.
- Solving cross-session pose-graph anchoring beyond the current SLAM session.
- Replacing the existing Nav2 stack or frontier explorer behavior.
- Refactoring unrelated `g1_nav` launch, behavior tree, or exploration code.

## Reference Compatibility

The reference behavior comes from the existing GO2W chain:

- `scripts/start_navigation_headless.sh`
- `scripts/navigation_executor.py`
- `docs/navigation_executor_zh.md`

The new `g1_nav` executor should preserve the same mental model:

- the robot is the WebSocket client
- the cloud/brain side is the WebSocket server
- navigation commands use flat JSON fields
- `navigate_to` uses `request_id`, `sub_id`, and `target_id`
- `abort_navigation` identifies the task by `request_id`
- the executor sends `on_progress`, `on_arrived`, and `on_error`
- heartbeats use `{"type": "ping"}` / `{"type": "pong"}`

New POI behavior should be added as a minimal protocol extension rather than a
new nested envelope format.

## Ownership Boundary

### Brain / Server Responsibilities

- natural-language understanding
- task orchestration
- semantic POI naming and ID allocation
- request lifecycle IDs such as `request_id` and `sub_id`
- deciding when to navigate, abort, or mark a POI

The brain may store semantic POI metadata such as:

- POI ID
- display name
- business tags
- domain meaning

The brain does not need to own live geometric pose data for POIs.

### Executor Responsibilities

- maintaining the WebSocket session to the brain/server
- resolving `target_id` into the current navigable pose
- sending Nav2 `NavigateToPose` goals and cancellation requests
- maintaining the local geometric POI store
- capturing current robot pose for `mark_current_poi`
- updating local POI geometry when loop closure changes the `map <- odom`
  transform

This split keeps semantic reasoning in the brain and geometric truth on the
robot, where the live SLAM state is available.

## Runtime Topology

The runtime stack is:

1. external SLAM publishes map and localization data
2. `g1_nav` runs Nav2 and robot-side motion execution
3. the new executor connects to the cloud WebSocket server
4. the server sends semantic commands such as `navigate_to POI_001`
5. the executor resolves that POI locally and drives Nav2
6. the executor reports progress and result events back to the server

For POI creation:

1. the brain chooses a semantic ID and name
2. the server sends `mark_current_poi`
3. the executor snapshots the current pose without interrupting navigation
4. the executor stores the local geometric POI record
5. the executor replies with success or error

## Chosen Approach

Implement the executor inside `g1_nav` as a layered Python subsystem that is
installed as a ROS 2 runnable script, following the same packaging style as the
existing GO2W executor.

The new components are:

- `g1_nav/navigation_executor.py`
  WebSocket session, command routing, state machine, heartbeat, reconnect
- `g1_nav/nav2_action_bridge.py`
  ROS 2 / Nav2 interaction and feedback translation
- `g1_nav/pose_and_poi.py`
  pose acquisition, POI normalization, local POI store, loop-closure
  reprojection

Tests should exercise protocol and POI logic without requiring a live Nav2
stack wherever possible.

## Packaging And Entry Point

The executor should be installed through `CMakeLists.txt` using
`install(PROGRAMS ...)`, matching the existing `g1_nav` pattern and the
reference `go2w_real` package style.

Expected invocation:

```bash
ros2 run g1_nav navigation_executor.py --server-uri ws://<server>:8100/ws/navigation/executor
```

This keeps operational usage close to the reference chain and avoids inventing a
second packaging style inside the same repo.

## Protocol

### Existing Navigation Messages

These messages intentionally preserve the flat GO2W style.

Navigate:

```json
{
  "action": "navigate_to",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "target_id": "POI_001"
}
```

Abort:

```json
{
  "action": "abort_navigation",
  "request_id": "req_20260402_001"
}
```

Progress:

```json
{
  "event_type": "on_progress",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "remaining_distance": 1.234,
  "status": "running"
}
```

Arrived:

```json
{
  "event_type": "on_arrived",
  "request_id": "req_20260402_001",
  "sub_id": 1
}
```

Error:

```json
{
  "event_type": "on_error",
  "request_id": "req_20260402_001",
  "sub_id": 1,
  "error_message": "Nav2 action server is not available within 10.0s"
}
```

Heartbeat:

```json
{"type": "ping"}
{"type": "pong"}
```

### POI List Synchronization

The original brain-side protocol included `update_poi_list`, but after the
ownership decision in this design, that message should no longer carry
geometric truth from the server to the executor.

Instead, `update_poi_list` becomes a semantic directory sync. The executor uses
it only for:

- semantic aliasing
- display metadata
- server-side enable/disable flags

But the executor's local geometric POI store remains authoritative for
navigation and loop-closure updates.

Recommended shape:

```json
{
  "action": "update_poi_list",
  "version": 3,
  "poi_list": [
    {
      "id": "POI_006",
      "name": "会议室门口"
    }
  ]
}
```

If the server still sends pose fields for backward compatibility, the executor
ignores them.

### Mark Current POI Request

The new request should also use flat JSON:

```json
{
  "action": "mark_current_poi",
  "request_id": "2c3054fd-6f4c-4fea-98a3-d75fca26f0d1",
  "sub_id": 1,
  "poi": {
    "id": "POI_006",
    "name": "会议室门口"
  }
}
```

### Mark Current POI Ack

```json
{
  "event_type": "on_mark_poi_ack",
  "request_id": "2c3054fd-6f4c-4fea-98a3-d75fca26f0d1",
  "sub_id": 1,
  "status": "accepted"
}
```

### Mark Current POI Success

Because geometry is executor-owned, the success event does not need to make the
server authoritative for pose data. The executor may return the recorded pose as
diagnostic metadata, but the brain should treat it as informational.

Recommended payload:

```json
{
  "event_type": "on_mark_poi_success",
  "request_id": "2c3054fd-6f4c-4fea-98a3-d75fca26f0d1",
  "sub_id": 1,
  "poi": {
    "id": "POI_006",
    "name": "会议室门口",
    "frame_id": "map",
    "position": {
      "x": 12.3,
      "y": 4.5,
      "z": 0.0
    },
    "yaw": 1.57,
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.707,
      "w": 0.707
    }
  }
}
```

Server-side logic should treat the pose fields as optional metadata rather than
the source of truth.

### Mark Current POI Error

```json
{
  "event_type": "on_mark_poi_error",
  "request_id": "2c3054fd-6f4c-4fea-98a3-d75fca26f0d1",
  "sub_id": 1,
  "error_message": "当前位置定位失败"
}
```

## Executor State Machine

The executor keeps the same high-level states as the reference implementation:

- `disconnected`
- `connecting`
- `idle`
- `starting`
- `navigating`
- `aborting`
- `error`

Behavior rules:

- receiving `navigate_to` preempts the current navigation task
- receiving `abort_navigation` cancels the matching task if it is active
- `update_poi_list` does not affect navigation state
- `mark_current_poi` does not interrupt or preempt navigation
- WebSocket disconnect triggers local cleanup and reconnect

### Mark POI During Motion

`mark_current_poi` is explicitly allowed while the robot is moving. This is
required for visual-model-assisted live labeling during motion.

The executor should therefore:

- accept `mark_current_poi` in `idle`, `starting`, and `navigating`
- avoid cancelling or pausing the active Nav2 goal
- run POI capture as a short side task

To avoid recording obviously stale localization data, the executor should still
require pose freshness:

- the pose timestamp must be recent, default `<= 0.5s`
- the pose must be resolvable in the global `map` frame

If pose freshness is violated, the executor returns `on_mark_poi_error`.

## Pose Acquisition

The executor should acquire robot pose as follows:

1. prefer TF lookup of `map -> base_link` or a configurable robot base frame
2. fall back to `/lightning/odometry` if TF is unavailable
3. fail if neither source provides a recent global pose

The normalized pose representation should include:

- `frame_id`
- `position.x/y/z`
- `yaw`
- quaternion orientation
- source timestamp

All externally visible POI poses should be normalized to the `map` frame.

## POI Data Model

### External Semantic Record

The semantic record exposed to the brain is:

- `id`
- `name`
- optional business metadata outside geometric pose ownership

### Local Geometric Record

The executor's local geometric record contains:

- `id`
- `name`
- current `map` pose
- tracking anchor in `odom`
- session metadata needed to decide whether reprojection is valid

Recommended local fields:

- `id`
- `name`
- `frame_id: "map"`
- `position`
- `yaw`
- `orientation`
- `tracking_frame: "odom"`
- `odom_position`
- `odom_yaw`
- `slam_session_id`
- `last_projected_map_pose`

The local store is authoritative for `target_id` resolution.

## Local POI Store

The executor should keep a local POI store file under robot-local control,
separate from brain-owned semantic memory.

The local store format is YAML, matching the existing waypoint convention from
the reference chain.

Default path:

- `~/.ros/g1_nav/poi_store.yaml`

The local POI store must support:

- atomic writes
- full reload on startup
- in-memory index by POI ID
- semantic updates without losing geometric fields

The executor should resolve navigation targets from this local store:

- `navigate_to.target_id` -> local POI geometric pose

If the target ID is unknown locally, the executor returns `on_error`.

## Loop Closure And POI Reprojection

### Why This Is Needed

A POI recorded during SLAM is not stable if only its raw `map` pose is stored.
When the SLAM backend applies loop closure, the `map <- odom` transform can
change, moving the robot and the environment in global map coordinates.

### Chosen Strategy

The executor should treat local POIs as:

- externally represented in `map`
- internally anchored in `odom` for the active SLAM session

When loop closure changes the `map <- odom` transform, the executor reprojects
all POIs anchored in the current session back into the new `map` frame.

### Update Trigger

The executor should continuously observe the `map <- odom` transform and
trigger reprojection only when the transform changes beyond thresholds.

Recommended defaults:

- translation threshold: `0.05 m`
- yaw threshold: `0.03 rad`

This avoids rewrite churn when TF jitter is small.

### Scope Limitation

This reprojection guarantee is only valid inside the current SLAM session.

It does not attempt to solve:

- robot restarts that reset `odom`
- switching to a different map
- durable cross-session pose-graph anchoring

Across restarts, the executor should load the latest saved `map` pose and start
fresh tracking only for POIs updated during the new session.

## Error Handling

Recommended user-facing error buckets include:

- invalid command payload
- unknown `target_id`
- POI ID or name conflict
- Nav2 action server unavailable
- Nav2 goal rejected
- Nav2 result failure
- Nav2 feedback watchdog timeout
- missing or stale pose data for `mark_current_poi`
- map or TF not ready
- local POI store write failure

`error_message` should remain readable text because it is surfaced to operators.
Structured internal error codes may be added later, but they are not required
for the first version.

## Testing Strategy

Testing should follow the same layering as the design.

### Unit Tests

Add Python tests for:

- message parsing and validation
- waypoint / POI loading
- `target_id` resolution
- semantic `update_poi_list` merge behavior
- `mark_current_poi` success payload formation
- loop-closure reprojection math
- stale-pose rejection

### Integration-Style Tests With Fakes

Add tests using fake bridge objects for:

- `navigate_to` -> progress -> arrived flow
- `navigate_to` -> error flow
- `navigate_to` preemption by a newer `navigate_to`
- `abort_navigation` -> paused flow
- `mark_current_poi` while idle
- `mark_current_poi` while navigating
- WebSocket disconnect cleanup behavior

### Manual Verification

Manual bringup should mirror the GO2W verification order:

1. bring up the local navigation stack
2. verify local direct navigation without WebSocket
3. run the new executor against a mock WebSocket server
4. validate `navigate_to`, `abort_navigation`, and heartbeat
5. validate `mark_current_poi` in both stationary and moving cases
6. validate a synthetic `map <- odom` shift and confirm local POI reprojection

## Implementation Outline

The implementation should proceed in this order:

1. add protocol and POI-store unit tests
2. add the new Python executor modules
3. wire the new script into `CMakeLists.txt`
4. add fake-bridge integration tests
5. verify the new tests pass

This order preserves TDD discipline and keeps protocol work isolated from
robot-specific runtime concerns.

## Open Operational Notes

- The server should not assume that any pose returned in
  `on_mark_poi_success` is authoritative beyond operator feedback.
- If future requirements need cloud-side map visualization, that should be a
  separate geometry-sync design rather than overloading the semantic control
  protocol.
- If the SLAM backend later exposes stable pose-graph node identifiers, the
  current `odom`-anchor strategy can be upgraded without changing the old
  navigation command format.
