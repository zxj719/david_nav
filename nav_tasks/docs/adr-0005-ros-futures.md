# ADR-0005: only the owning runtime spins ROS

On 2026-09-08 the old dock helper used `spin_until_future_complete` while its
node was already owned by a MultiThreadedExecutor. A second/global executor
registered the same action client's waitable into another wait set, producing
an action status-subscription index error and process exit. The new adapters
must never spin a second executor while waiting for a future.

`adapters/ros/futures.py` provides the two waiting modes: callback forwarding
through a thread-safe concurrent future into asyncio, and an event-based wait
for the legacy threaded dock caller. Canceling an asyncio waiter does not cancel
the source ROS future, which may still deliver the handle needed for cleanup.
Late source errors are retrieved even after a waiter has gone away.

The action session receives feedback through `loop.call_soon_threadsafe`; all
ownership and user feedback callbacks run on the one task loop. This is not an
R6 executor-thread architecture switch: the old runtime spin loop is untouched.
Its measurement and any polling-interval decision remain separately tracked.

Server readiness uses nonblocking `server_is_ready` probes with asyncio interval
waits, only while an attempt is being admitted. It must not use the blocking
`ActionClient.wait_for_server` loop on the task loop, because that prevents the
receive loop from processing an abort during startup. A fresh goal message and
timestamp are constructed only after readiness, on every attempt.
