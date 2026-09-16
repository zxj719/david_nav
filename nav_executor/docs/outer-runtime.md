# Outer runtime and graceful exit

`ros2 run nav_executor navigation_executor --help` prints the frozen legacy help
without loading configuration or initializing ROS. The new executable is separate
from the unchanged david_nav executable and production aliases. It currently
supports ordinary POIs only: configured Dock IDs fail before ROS/storage opens.
Do not deploy it as a replacement for a Dock-enabled production entry yet.

The CLI resolves configuration, then installs main-thread INT/TERM/HUP handlers.
The first signal sets an asyncio stop event and determines exit code 128 + signal;
later signals do not interrupt cleanup. Embedding code should call `app.runtime.run`
with its own stop event instead of using the CLI signal helper. Previous handlers
are restored after the runner finishes. Startup failures return 1 after cleanup.

RosHost owns a private Context, Node, SingleThreadedExecutor and TF listener.
Context.init is used directly so rclpy does not install context-shutdown signals.
The listener does not spin its own thread. One strongly owned asyncio task polls
the explicit executor at the existing 100 Hz cadence; this is not an R6 performance
improvement claim. Task implementation is accessed only through nav_tasks.api.

Shutdown order is:

1. Stop and join WS ingress/connection workers, event pump and status reporting;
   unsubscribe the command adapter from task events.
2. Close the task runtime (or retry its failed-construction rollback), preserving
   ROS callback processing while cancellation waits for action acknowledgement and
   terminal result. A cancel deadline does not prove that motion stopped.
3. Join the ROS polling task, shut down its executor, unregister TF, destroy the
   node, then shut down only the private context.

The cleanup task is strongly owned and shielded from caller cancellation. Failed
cleanup retries and logs `cleanup pending`; resources are not abandoned after a
fixed deadline. A callback exception stops ingress but continues callback attempts
while cleanup settles. Partial construction stages are retained for teardown.
No task completion frame is invented during shutdown, and no outbox flush,
exactly-once delivery or crash persistence guarantee is introduced.

Humble Executor.shutdown can return False after setting its internal shutdown
flag; a second call can then return True without checking remaining callbacks.
RosHost therefore treats the first False as a sticky unproven state and retains
the node/context. A lost ROS context or this state can require manual recovery;
repeated signals intentionally do not force release. Tests inject this failure,
but it is not evidence that a physical robot is stopped. No operator escalation
or kill policy is added here.

Pure tests cover normal/error/construction exits, late action results, repeated
caller cancellation, cleanup retry and signal restoration. Opt-in sandbox checks
use domain 99, the explicit DDS environment and owned subprocesses; they send only
WS ping, never ROS goals or velocity commands. They are excluded from colcon test
collection and refuse to start while a user stack is detected. The installed
websockets version/path is logged; no library monkeypatch or package install occurs.
