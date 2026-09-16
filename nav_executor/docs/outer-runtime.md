# Outer runtime and graceful exit

`ros2 run nav_executor navigation_executor --help` prints the frozen legacy help
without loading configuration or initializing ROS. The new executable is separate
from production aliases. The retained david_nav executable now forwards here in
the refactor workspace. The equivalent
`nav_task_manager` entry composes ordinary POIs and opt-in Dock in one process.
Supply a complete `--dock-config-file` to enable Dock; Dock IDs without it are
rejected before ROS/storage opens. `--no-websocket` disables WS imports, connection,
proxy changes and WS observers. Default WS behavior and frozen help are unchanged.
Both retained david_nav programs are thin shims. The standalone dock_navigator
forwards to app.dock_cli, selects installed david_config/dock/standalone.yaml
unless --dock-config-file is explicit, and forces --no-websocket. Its sixteen
policy values (5-degree yaw, two retries) are preserved as data. R8 changes only
the default goal topic to /dock_goal, avoiding Nav2 /goal_pose competition.
ROS parameters still override them; explicit topic overrides are not blocked.
The old precise launch now also has only one shared owner: WS-enabled Dock waits
for Nav2, ROS-only Dock keeps its former core/chassis-delay timing. Its eight ROS
overrides retain /dock_goal, 0.4363 yaw and zero retries. Missing WS prerequisites
or a detected existing task owner refuse that launch instead of starting a partial
second Dock runtime; no user process is signaled. The pgrep check is conservative,
not an atomic global singleton lock. Do not mix independent legacy/manual owners.
Direct old executor CLI use with --dock-poi-id now requires the complete
--dock-config-file policy; the compatibility precise launch supplies it. External
cross-process Dock routing is replaced by the shared owner, not silently retained.
Production aliases and deployed files have not changed.
The prepared navrun graph now selects this manager: precise+WS uses one deferred
process for both ingress paths, precise without WS uses one core-phase Dock node,
and tour without WS omits it. Its argv carries a mode-merged robot profile snapshot
and, for precise mode, the complete Dock parameter file. Missing snapshots reject
launch rather than falling back to installed David data. Dock in precise+WS starts
after Nav2 readiness/full startup checks; it is no longer a separate early node.

The CLI resolves configuration, then installs main-thread INT/TERM/HUP handlers.
The first signal sets an asyncio stop event and determines exit code 128 + signal;
later signals do not interrupt cleanup. Embedding code should call `app.runtime.run`
with its own stop event instead of using the CLI signal helper. Previous handlers
are restored after the runner finishes. Startup failures return 1 after cleanup.

RosHost owns a private Context, Node, executor and TF listener. Ordinary mode
keeps SingleThreadedExecutor and node `navigation_executor`. Dock uses node
`dock_navigator` and MultiThreadedExecutor, retaining Humble's CPU-count default
with a minimum of two threads so blocking action execution cannot starve its
action/TF callbacks. The sixteen Dock parameters are declared before task assembly;
effective ROS overrides are validated and audited. Resolved/remapped ingress and
backend action names must differ, preventing recursive self-routing.
Context.init is used directly so rclpy does not install context-shutdown signals.
The listener does not spin its own thread. One strongly owned asyncio task polls
the explicit executor at the existing 100 Hz cadence; this is not an R6 performance
improvement claim. Task implementation is accessed only through nav_tasks.api.

Shutdown order is:

1. Seal Dock ingress; stop and join WS workers, event pump and status reporting;
   unsubscribe the command adapter from task events.
2. Await Dock ingress.close and task-runtime.settle with ROS/asyncio alive. These
   seal task admission and join owned motion/worker/session cleanup but retain ROS
   clients/subscriptions. Failed-construction rollback is retried with ROS alive.
3. Join the ROS polling task, then await host.quiesce off the asyncio thread.
   It joins SDK callbacks, the callback pool, and already-queued result-send tasks.
   Their final loop calls can still run; no new DDS work is taken during draining.
4. Close the settled runtime and destroy Dock ingress, retaining failed stages
   for retry. Unregister TF, destroy the node, then close only the private context.

Ordinary-only mode retains its previous close/spin-join/host-close sequence.
The public settle phase is optional for single-thread callers; close includes it.

The cleanup task is strongly owned and shielded from caller cancellation. Failed
cleanup retries and logs `cleanup pending`; resources are not abandoned after a
fixed deadline. A callback exception stops ingress but continues callback attempts
while cleanup settles. Partial construction stages are retained for teardown.
No task completion frame is invented during shutdown, and no outbox flush,
exactly-once delivery or crash persistence guarantee is introduced.

Humble Executor.shutdown can return False after setting its internal shutdown
flag; a second call can then return True without checking remaining callbacks.
RosHost therefore treats the first False as a sticky unproven state and retains
the node/context. Dock quiescence also treats exceptions, a suspended/failed queued
task or an unsupported SDK layout as unproven, never retrying SDK shutdown as proof.
A lost ROS context or this state can require manual recovery;
repeated signals intentionally do not force release. Tests inject this failure,
but it is not evidence that a physical robot is stopped. No operator escalation
or kill policy is added here.

Pure tests cover normal/error/construction exits, late action results, repeated
caller cancellation, cleanup retry and signal restoration. Opt-in sandbox checks
use domain 99, the explicit DDS environment and owned subprocesses; they send only
WS ping, never ROS goals or velocity commands. They are excluded from colcon test
collection and refuse to start while a user stack is detected. The installed
websockets version/path is logged; no library monkeypatch or package install occurs.

## SDK compatibility boundary

David's installed Humble MultiThreadedExecutor has no shutdown override. Its base
work tracker does not join the ThreadPoolExecutor, and Task/Future completion can
enqueue a separate ActionServer result-send task after an execute callback returns.
The adapter therefore uses the audited `_executor`, `_pending_tasks` dictionary
and `_tasks_lock` layout. After joining the pool it invokes remaining SDK Tasks on
the shutdown helper thread; completion-created tasks are drained too. Unknown
layouts fail closed. This binding must be revalidated on SDK upgrades; it is not
a claim of compatibility with every Humble distribution. An installed-SDK pure
Task/Future probe verifies this scheduling behavior without initializing ROS.

The drain only covers work already owned by the executor, not DDS requests still
on the wire, a future client's get-result request, or remote receipt/ACK. It is not
an outbox flush or proof of physical braking. No-goal real-host acceptance remains
deferred while the user's localization/radar/IMU stack is running.
