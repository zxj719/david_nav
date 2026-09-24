# Outer runtime and graceful exit

`nav_task_manager` is the single process composition root for ordinary POIs and
optional Dock ingress. Supply a complete `--dock-config-file` to enable Dock;
`--no-websocket` disables WS imports, connections and observers. Incomplete Dock
policy is rejected before ROS or storage opens.

`RosHost` owns a private context, node, executor and TF listener. Ordinary mode
uses a single-threaded executor. Dock uses a multithreaded executor so a blocking
action callback cannot starve action/TF callbacks. Task implementation is reached
only through `nav_tasks.api`.

Shutdown order is contractual:

1. Seal ROS/WS ingress and stop accepting work.
2. Settle the task runtime and join owned motion, worker and session cleanup while
   ROS callbacks are still available.
3. Stop ROS polling, then drain already-owned SDK callbacks.
4. Close the settled runtime, destroy ingress and TF resources, then close the
   private context.

Cleanup is strongly owned and shielded from caller cancellation. Failed cleanup
remains visible and retryable; it is never reported as idle or physical stop.
Signals select exit code `128 + signal` but do not bypass that order.

Humble executor internals used for callback draining are an explicit SDK
compatibility boundary and must be revalidated on ROS upgrades. The drain covers
already-owned work, not DDS data still on the wire, remote acknowledgement or
physical braking.
