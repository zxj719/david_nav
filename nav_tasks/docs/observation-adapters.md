# Navigation observation ports

`adapters.ros.tf_pose_source.TfPoseSource` implements PoseSource against an injected
TF buffer and one owned odometry subscription. Every frame/topic is explicitly
supplied by the assembly root. Message header frame overrides and empty-header
fallbacks, latest-time queries, sample height/stamp and depth-10 QoS match the old
provider. The topic is not automatically changed to another localization output.
The adapter never starts/stops localization or owns a node, TF listener or executor.

`adapters.ros.grid_source.RosGridSource` owns only the two configured depth-10
OccupancyGrid subscriptions. `snapshot` captures their current message references
under a lock, then performs the same axis-aligned GridMapView conversion. Feed
this callable to `GridGoalResolver`; search policy stays in the pure core. Missing
maps still use the existing normal-navigation allowance. **This is not the dock
footprint safety check**: dock must keep its separate conservative missing-map,
out-of-bounds and occupied-footprint rules, including rotated map origins.

Both adapters import ROS/message types only during construction when no injected
type/factory is supplied. They create no background task or ROS spin loop. Callback
state is lock-protected across ROS threads; close must run on the resource-owner
thread after motion consumers have stopped. Close seals callbacks before removing
only owned subscriptions; repeated successful close is a no-op and failed removal
retains its handle for retry. Partial grid construction removes the first
subscription if the second fails. The assembly root remains responsible for
retaining and retrying the RuntimeAssemblyError cleanup handle if both creation
and rollback fail; external node cleanup must not bypass owned handles. No
localization process is ever destroyed by these methods.

## Compatibility choices

- The pose adapter uses `domain.geometry.legacy_yaw_from_quaternion`, a direct
  relocation of the old formula. The general normalized helper changes rounding,
  the positive-pi boundary and invalid/nonunit inputs; applying it here would
  change unrounded POI wire fields and stored data. Eight seeds cover 832
  quaternion inputs with exact binary-float comparisons to the old function;
  captured old/new POI success frames also match byte-for-byte. The general
  normalized helper remains the default for new geometry. No new invalid-IMU or
  quaternion policy is claimed; changing it requires a separately audited fix.
- TF lookup failures become PoseUnavailableError with unchanged message and
  original exception cause. The status formatter uses that cause's class name,
  preserving the legacy `pose=n/a (...)` marker. Missing odometry keeps its old
  text. Map/odometry capture still uses independent latest samples, not a new
  synchronization or odometry-freshness guarantee.
- `app.loop_closure.watch_loop_closure` is a coroutine for the existing supervisor,
  not a new process/task creator. It keeps exact transform equality detection,
  configured polling, 0.05 m / 0.03 rad defaults, missing-TF retry and propagation
  of repository/session failures. It remembers only successfully handled TF.
  Session changes with identical TF remain skipped as in the old watcher; a
  new session/reset policy is not bundled into migration.

Tests inject ROS I/O and never initialize ROS or publish a goal/velocity. A full
new protocol→manager→skill→backend chain now uses these actual observation
adapters, GridGoalResolver and YAML storage; only node, messages, TF buffer and
action I/O are fakes. This proves port composition in memory, not DDS discovery,
real TF timing, robot stopping or complete runtime teardown.

The refactor runtime now assembles these adapters with WS lifecycle, shared
Dock/Spin, event production and compatibility entry shims; see
[runtime ownership](runtime.md). Production is not deployed by this integration.
Remaining P6/P7 gates and physical acceptance are separate from the in-memory
composition and no-goal lifetime checks.

## Snapshot and teardown limits

The ordinary grid adapter stores incoming message references until snapshot time;
successive snapshots may therefore observe mutations to the same input message.
The converted cell list is copied but the resulting GridMapView is itself mutable.
Dock instead copies its retained map at callback time, including origin rotation.
Neither policy synchronizes map and costmap timestamps or adds freshness checks.

Only TF lookup failures are normalized. Field decoding after a successful lookup
can raise directly; Dock does not turn such malformed responses into a logged
missing-pose result. Capture uses independent map/odometry times, and cached
odometry records are detached from the original message. New pure tests preserve
these boundaries rather than introducing a new input-validation policy.

Grid close removes subscriptions in order and stops at the first failure. Already
removed handles are not retried, while failed and unattempted handles remain owned.
Callbacks are sealed even if removal fails. The lifecycle owner must serialize
close calls and keep the external node alive until cleanup succeeds.
