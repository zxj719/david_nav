# ADR-0008: supervised crash cleanup precedes ownership release

Status: implemented for P6.5 R2 / SPEC §4.5. Production remains unchanged.

## Decision

Skill exceptions and invalid outcome contracts escape the worker into the existing
TaskSupervisor crash boundary. The manager matches the actual current worker's
name (including admission generation) and exception identity, reports one FAILED
event with `执行器内部错误: <异常类型>`, and spawns strongly owned cleanup. The WS
adapter encodes that event as the existing on_error frame; it is not on_arrived.
Normal typed skill failures and the legacy external-CANCELED G12 pause are unchanged.

The callback is synchronous and never waits for motion. A separate supervised
operation takes the same transition lock as cancel/preempt/close and checks the
active object identity again. A queued cleanup cannot cancel a replacement even
when task IDs are reused. Feedback is sealed as soon as the worker exits.

Reporting an error does not prove motion stopped. Ownership remains until backend
cancel and worker termination are confirmed. Only then clear the active task, set
IDLE and issue one FAILED TaskCompletion with the same error detail. A competing
cancel/close/preempt may perform that cleanup first; preserve the original FAILED
receipt and do not append a paused event after the crash's error. Normal explicit
cancellation still settles CANCELED and emits its existing paused event.

A known worker exception is the original crash, not a second cleanup failure.
Ignore that exact exception only when it was recorded before this cleanup attempt;
backend cancellation errors always retain ERROR ownership. A new worker exception
raised during explicit cancellation still fails that attempt. The supervisor
records it for a later explicit cleanup retry without inventing a competing crash
terminal event after cancellation has already won. No automatic tight retry loop
is added: failed recovery retains ownership and permits cancel/preempt/close retry.
Canceling an API waiter cannot abandon supervised cleanup. Close seals admission
and waits behind the same cleanup barrier; completion observers remain isolated.

## Evidence and limits

Red tests reproduced the old raw error message and retained zombie task. Pure
manager tests cover delayed and failed cleanup, retry by abort/close/preempt,
same-ID replacement, canceled shutdown waiters, stale recovery and fresh exceptions
during cancellation. The actual WS adapter/POI skill with a fake navigate backend
checks exact on_error bytes, IDLE/no active task after cleanup, and successful
subsequent navigation. Existing ROS-ingress and valid wire-golden tests also run
with fake I/O. These checks do not send ROS goals or prove physical stopping.

If backend cleanup cannot prove release, R2 deliberately does not claim IDLE.
That is the existing backend safety contract, not permission to discard an owned
goal. Operational recovery for permanently unresponsive hardware remains manual.
