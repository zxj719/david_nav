# WebSocket connection lifetime

`adapters.transport.ws_client.reconnect` receives the process-lifetime command
adapter/outbox and an injected asynchronous connection factory. The outer root
owns this coroutine, the task-event pump and status loop. No task manager, ROS
node, transport library monkeypatch or socket is created by importing the module.
`WsOptions` requires the URI and retains the old 30 s heartbeat/3 s retry defaults.

Each connection owns receive, heartbeat and outbox-drain tasks, keeps strong
references and cancels/joins all children before leaving the connection context.
The callbacks start/stop the runtime's connection-scoped loop-closure observer.
Disconnect never cancels motion, detaches the command adapter or replaces the
outbox. Navigation events generated while offline must continue to reach that
same outbox through the outer root's persistent pump.

The old runtime could leave a failed sender unnoticed while receive waited for
another frame. The extracted connection owner observes every child: sender or
heartbeat failure wakes it and triggers reconnection immediately. This lifecycle
correction does not change task retry policy, action cancellation or wire frames;
tests explicitly cover failure while receive is blocked and retained-frame replay.
If receive and send finish together, all results are retrieved during cleanup.

Heartbeat ping JSON, inbound ping/pong handling, silence-warning thresholds,
connection/retry log wording and attempt-one/every-tenth failure logging retain
their previous formats. In-flight count is exposed separately from total pending
count to preserve the old reconnect banner. Only JSON/non-object/ValueError
messages are ignored in this step; R1's broader malformed administrative-command
handling remains a separate change. No protocol ACK or exactly-once delivery is
introduced. See [task-adapter.md](task-adapter.md) for outbox limitations.

`proxy.bypass_proxy_for_server` explicitly appends the configured hostname to
both `no_proxy` and `NO_PROXY`, preserving existing hosts and idempotence. It is
not called on import; the robot-aware outer root chooses when to apply it.

Tests use in-memory sockets and the actual public task factory with fake ROS I/O.
An additional localhost-only test uses the installed WebSocket library and an
ephemeral port to exercise mark, navigate, abort and reconnect through actual
task/storage/backend implementations; the action client remains in memory, with
no ROS initialization or physical navigation goals. CLI/help, outer ROS lifetime,
Dock/Spin integration and the legacy executable switch are still pending.
