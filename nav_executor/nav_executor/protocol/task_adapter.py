"""Translate the existing wire commands to public task/POI APIs only."""

from typing import Callable

from nav_tasks.api import (
    ClosedError, EventKind, NavTaskError, PoiApi, PreemptPolicy, TaskApi, TaskEvent, TaskRequest,
)

from nav_executor.protocol.codec import (
    build_arrived_event, build_error_event, build_mark_poi_ack, build_mark_poi_error,
    build_mark_poi_success_event, build_progress_event, parse_command,
)
from nav_executor.protocol.messages import (
    AbortNavigationCommand, CommandParseError, MarkCurrentPoiCommand, NavigateToCommand,
    UpdatePoiListCommand,
)


def event_to_frame(event: TaskEvent) -> dict:
    """Encode a task event with the unchanged ordered wire builders.

    Args:
        event: Immutable domain event from TaskApi.

    Returns:
        One WS v1 mapping, ready for the reliable outbox.

    Raises:
        ValueError: An unsupported domain event is supplied.
    """
    if event.kind == EventKind.PROGRESS:
        return build_progress_event(
            event.task_id, event.sub_id, event.remaining_distance, event.status)
    if event.kind == EventKind.SUCCEEDED:
        return build_arrived_event(event.task_id, event.sub_id)
    if event.kind == EventKind.FAILED:
        return build_error_event(event.task_id, event.sub_id, event.detail)
    raise ValueError(f'unsupported task event: {event.kind}')


class TaskCommandAdapter:
    """Translate ingress and observe events without owning navigation policy.

    Keep one adapter across transport reconnects. A runtime-owned supervisor
    runs pump_events into the reliable outbox; all handle_message return frames
    must enter that same outbox. This class starts no background tasks itself.
    """

    def __init__(self, tasks: TaskApi, pois: PoiApi) -> None:
        """Subscribe on the task loop without creating clients or storage.

        Args:
            tasks: Public motion ownership API.
            pois: Public administrative POI API.
        """
        self._tasks = tasks
        self._pois = pois
        self._events = tasks.subscribe_events()
        self._closed = False

    async def handle_message(self, payload: dict) -> list[dict]:
        """Translate a decoded command without waiting for navigation completion.

        Args:
            payload: Decoded WS mapping; malformed fields are rejected before API calls.

        Returns:
            Immediate parse/admission/mark frames. Navigation progress and terminal
            frames, including abort confirmation, arrive through pump_events.

        Raises:
            ClosedError: The adapter was detached during runtime shutdown.
            CommandParseError: A malformed request lacks recoverable request identity.
            ValueError: Unsupported commands or invalid administrative outcomes.
            Exception: Storage/cleanup failure, isolated at the transport receive boundary.
        """
        if self._closed:
            raise ClosedError('task command adapter is closed')
        try:
            command = parse_command(payload)
        except CommandParseError as error:
            if error.request_id is None:
                raise
            return [build_error_event(error.request_id, error.sub_id, str(error))]
        if isinstance(command, NavigateToCommand):
            request = TaskRequest(
                command.request_id, command.action, {'target_id': command.target_id},
                command.sub_id, PreemptPolicy.ALLOW, source='websocket')
            try:
                await self._tasks.submit(request)
            except NavTaskError as error:
                return [build_error_event(command.request_id, command.sub_id, str(error))]
            return []
        if isinstance(command, AbortNavigationCommand):
            await self._tasks.cancel(command.request_id)
            return []
        if isinstance(command, UpdatePoiListCommand):
            pairs = [(str(item['id']), str(item['name'])) for item in command.poi_list]
            self._pois.update_directory(pairs)
            return []
        if isinstance(command, MarkCurrentPoiCommand):
            frames = [build_mark_poi_ack(command.request_id, command.sub_id)]
            result = await self._pois.mark(command.poi_id, command.poi_name)
            if result.error is not None:
                frames.append(build_mark_poi_error(
                    command.request_id, command.sub_id, result.error))
            elif result.poi is not None:
                frames.append(build_mark_poi_success_event(
                    command.request_id, command.sub_id, result.poi))
            else:
                raise ValueError('POI mark returned neither a POI nor an error')
            return frames
        raise ValueError(f'unsupported command: {command.action}')

    async def next_event(self) -> dict:
        """Encode the next WS/legacy-source task event without sending it directly.

        Other ingress events stay visible to general TaskApi subscribers, but are
        not responses to agent-server requests and must not enter the WS outbox.

        Returns:
            One event mapping for the reliable outbox.

        Raises:
            ClosedError: The adapter has already been detached.
        """
        if self._closed:
            raise ClosedError('task command adapter is closed')
        while True:
            event = await self._events.get()
            # Shared task observability includes ROS ingress, but its generated
            # identity is not an agent-server request. Empty source retains the
            # original direct-API/protocol compatibility contract.
            if event.source in ('', 'websocket'):
                return event_to_frame(event)

    async def pump_events(self, emit: Callable[[dict], None]) -> None:
        """Forward task events into a synchronous reliable-outbox enqueue callback.

        The runtime supervisor owns this coroutine. There is no await between
        removing an event from the subscription and transferring it to emit.

        Args:
            emit: Synchronous enqueue function, normally ReliableOutbox.put.
        """
        while True:
            emit(await self.next_event())

    def close(self) -> None:
        """Detach after stopping event consumers; never cancel motion on disconnect.

        The runtime must first cancel/join its supervised pump and other event
        waiters. Transport disconnect alone must not close this process-lifetime
        adapter or discard navigation events produced during the disconnect.
        """
        if not self._closed:
            self._tasks.unsubscribe_events(self._events)
            self._closed = True
