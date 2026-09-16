# ADR-0007: malformed messages do not own the connection lifetime

Status: implemented for P6.5 R1 (SPEC §4.7). R2/R3/R8 remain separate changes.

## Context

A missing `poi` or `request_id` raised KeyError past the old ValueError catch,
tearing down a healthy connection. Directory conversion used a lazy generator:
a later malformed item could fail after an earlier name had mutated in memory.
Invalid binary UTF encoding could also escape JSON decoding.

## Decision

The I/O-free decoder exposes ProtocolError with request/subtask context under
NavExecutorError, retaining ValueError ancestry. MalformedCommandError also derives
from the old CommandParseError, so compatibility imports and navigation error
handling remain valid. UnsupportedActionError is a sibling: unknown actions are
still logged and ignored, even when they carry a request ID.

Missing fields, invalid object/list/scalar shapes and failed numeric conversions
become MalformedCommandError. Recoverable IDs use the existing ordered on_error
frame; no context means log and discard. Navigate error wording and valid-command
coercions remain compatible: scalar identifiers can still be stringified, mark
fields are trimmed, directory values retain their whitespace, and invalid
navigation sub_id falls back to zero. Nonfinite integer conversion cannot escape.

Every directory item must be an object with id and name. Validate the complete
list and materialize converted pairs before invoking PoiApi. A malformed batch
is rejected in full; it does not partially rename known IDs. In particular, an
unknown ID missing a name now makes the batch malformed instead of relying on
the old repository's membership-before-name lookup. A valid unknown ID is still
ignored without creating a pose. This is an explicit R1 malformed-input change,
not a change to valid catalog semantics or a storage transaction guarantee.

The receive boundary catches individual decode/handler exceptions, logs and
continues. JSON/non-object/unsupported logs retain their prior text; new malformed
and unexpected-failure logs identify the isolated error. The adapter returns
correlated malformed responses through the existing reliable outbox. No ACK,
new wire action or fabricated terminal outcome for unexpected handler failures
is introduced. Existing application side effects are not automatically undone;
task crash cleanup is R2 and storage I/O rollback is not provided by this repair.

Socket iteration and ping/heartbeat/outbox sends remain outside the application
failure boundary so real connection failures reconnect. BaseException, including
asyncio.CancelledError, still propagates for shutdown.

## Evidence and limits

Red tests reproduced malformed decoder escapes, handler failure propagation,
partial catalog mutation and real-loopback reconnect. Green coverage uses the
actual parser/adapter/public runtime with temporary storage and fake ROS/action
I/O. A real ephemeral localhost WebSocket receives malformed mark, missing abort
ID, invalid JSON, non-object JSON and invalid UTF bytes, then completes a valid
navigation request on exactly one connection. Existing wire goldens remain.
No ROS context, external navigation goal or physical robot acceptance is implied.
