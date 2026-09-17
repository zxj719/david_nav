# ADR-0005: only the owning runtime spins ROS

On 2026-09-08 the old dock helper used `spin_until_future_complete` while its
node was already owned by a MultiThreadedExecutor. A second/global executor
registered the same action client's waitable into another wait set, producing
an action status-subscription index error and process exit. The new adapters
must never spin a second executor while waiting for a future.

Implementation: [future bridge](../nav_tasks/adapters/ros/futures.py),
[action ownership](../nav_tasks/adapters/ros/action_session.py),
[current host](../../nav_executor/nav_executor/adapters/ros/host.py).
The archived cancellation comment below describes the old wrapper's ownership;
the current adapter additionally retrieves late source errors and retains handles
needed for cleanup. Keeping that original text is not an instruction to drop
errors or abandon action ownership in the new runtime.

`adapters/ros/futures.py` provides the two waiting modes: callback forwarding
through a thread-safe concurrent future into asyncio, and an event-based wait
for the legacy threaded dock caller. Canceling an asyncio waiter does not cancel
the source ROS future, which may still deliver the handle needed for cleanup.
Late source errors are retrieved even after a waiter has gone away.

A source can also store asyncio.CancelledError as its exception without setting
its canceled flag. The callback now forwards that exception through the same
local bridge as ordinary exceptions; it must not escape the completing thread
or leave a registered waiter pending. An abandoned waiter still causes the late
source error to be retrieved without canceling the source or writing into an
already-completed local bridge. Five standard-library-only cases cover ready,
loop-thread, worker-thread and abandoned-waiter completion. This is a Future
notification fix, not proof of motion cleanup or physical stopping.

The action session receives feedback through `loop.call_soon_threadsafe`; all
ownership and user feedback callbacks run on the one task loop. This is not an
R6 executor-thread architecture switch: the old runtime spin loop is untouched.
Its measurement and any polling-interval decision remain separately tracked.

Server readiness uses nonblocking `server_is_ready` probes with asyncio interval
waits, only while an attempt is being admitted. It must not use the blocking
`ActionClient.wait_for_server` loop on the task loop, because that prevents the
receive loop from processing an abort during startup. A fresh goal message and
timestamp are constructed only after readiness, on every attempt.

## 背景原文（逐字保留）

下列片段取自标明的不可变提交，保留原标点、缩进和时间。它们不是本轮新的现场测量，
也不是实体停车/到达验收；存在与现状不同的历史描述时，以本 ADR 上方当前决定为准。

### dock-future

来源：`93676a5:david_nav/david_nav/dock_navigator.py`。
SHA256：`4447b8fdd8c50882f9162a7884e12ab3b3dada7a658e24e8080ff8671bfcef5e`。

<!-- history: dock-future -->

```python
        """等 future 完成，但**自己绝不 spin** —— 由 main() 里的执行器负责转。

        ⚠ 这里以前用的是 rclpy.spin_until_future_complete(self, fut)。那个函数会把
        本节点 add_node 到 rclpy 的**全局 SingleThreadedExecutor**，而 main() 里的
        MultiThreadedExecutor 同时还持有这个节点、并且已经把 action client 的实体
        注册进了它自己的 wait set。两个执行器对同一个 action client 各建一套
        wait set，取就绪实体时下标对不上，直接抛：

            RCLError: Failed to get number of ready entities for action client:
            wait set index for status subscription is out of bounds,
            at ./src/rcl_action/action_client.c:623

        进程 exit 1 —— 2026-09-08 实机第一个 dock 目标就是这么死的：bt_navigator
        刚打印 "Begin navigating" 100ms 后就变成 "Client requested to cancel"，
        rviz 里能看到路径、车一步不动。

        换成事件等待：future 完成回调由本节点自己的执行器触发，这里只阻塞在
        threading.Event 上，不再碰任何 wait set。返回 True=完成，False=超时。
        """
```

### bridge-cancel

来源：`93676a5:david_nav/david_nav/nav2_action_bridge.py`。
SHA256：`214debc433a28f257fc88ae8a8cfff95cd4bb4cf65befa20ebe632aa69b7b541`。

<!-- history: bridge-cancel -->

```python
            # abort(task.cancel)后 wrapped 已被 wrap_future 连带取消,rclpy
            # future 稍后完成时 set_result 会炸 InvalidStateError 并打进
            # rclpy spin —— 静默丢弃即可(结果没人要了)。
```
