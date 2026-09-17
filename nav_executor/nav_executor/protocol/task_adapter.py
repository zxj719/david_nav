"""仅通过公共任务和 POI API 转换既有线上命令，不依赖任务内部实现."""

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
    """按既有有序构造器编码任务事件，不自行判定运动是否完成.

    Args:
        event: TaskApi 提供的领域事件，成功来源由任务层负责。

    Returns:
        一个可交给发送队列的 WS v1 字典。

    Raises:
        ValueError: 事件类型不是进度、成功或失败，或进度距离不能转为浮点数。
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
    """转换入口命令并观察任务事件，不持有导航策略或自行启动后台任务.

    传输重连期间复用同一适配器。运行器监督 pump_events，并将它产出的事件
    及 handle_message 返回的全部帧交给同一发送队列；断连不应丢弃后台导航事件。
    """

    def __init__(self, tasks: TaskApi, pois: PoiApi) -> None:
        """在任务事件循环订阅事件，不创建 ROS 客户端或 POI 存储.

        Args:
            tasks: 提供运动所有权管理和事件订阅的公共任务 API。
            pois: 公共 POI 管理 API。
        """
        self._tasks = tasks
        self._pois = pois
        self._events = tasks.subscribe_events()
        self._closed = False

    async def handle_message(self, payload: dict) -> list[dict]:
        """解析命令并调用公共 API，不等待导航终态；标记 POI 则等待 mark 结束.

        navigate_to 等待准入，abort 等待 cancel API 返回，目录更新同步执行。
        标记 ACK 先在局部列表构造，但 mark 返回后才与结果一起交付调用方；
        mark 抛出异常时不会返回这条已构造 ACK，不能把此处称为立即发送 ACK。

        Args:
            payload: 已解码 WS 字典，调用业务 API 前完成协议字段解析。

        Returns:
            错误或标记应答列表；正常导航准入、取消与目录更新返回空列表。
            导航进度和终态通过独立事件泵转发，不由本返回值证明取消完成。

        Raises:
            ClosedError: 入口检查时适配器已关闭。
            CommandParseError: 坏命令没有可恢复的请求身份。
            ValueError: 动作不支持，或标记结果既没有 POI 也没有错误。
            Exception: 未被此层转换的业务或存储异常，由收包边界按既有策略隔离。
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
        """等待并编码 WS 或旧空来源事件，不直接发送.

        其它入口事件仍供一般 TaskApi 订阅者观察，但不作为 Agent 请求应答进入 WS。
        只在进入方法时检查关闭标志；关闭不能唤醒已等待的 get，所有者须先停止等待者。

        Returns:
            一个交给发送队列的协议事件字典。

        Raises:
            ClosedError: 进入方法时适配器已关闭。
            ValueError: 选中的领域事件不能转换为支持的协议事件。
            asyncio.CancelledError: 外层取消事件等待。
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
        """持续将任务事件交给同步入队回调，由外层运行器监督本协程.

        从取得事件到调用 emit 之间没有额外 await；这不是持久交付事务，
        若转换或 emit 失败，不会在此把已取出的事件放回订阅队列。

        Args:
            emit: 同步入队函数，通常为 ReliableOutbox.put。

        Raises:
            Exception: 事件读取、转换或回调失败，交给运行器处理。
            asyncio.CancelledError: 外层取消事件泵。
        """
        while True:
            emit(await self.next_event())

    def close(self) -> None:
        """在事件消费者停止后解除订阅，不因传输断开而取消运动.

        运行器须先取消并等待事件泵及其它等待者；仅断连不能关闭本进程生命周期对象。
        解除订阅成功后才设置关闭标志，可重复调用；解除失败则传播异常并保留未关闭状态。
        """
        if not self._closed:
            self._tasks.unsubscribe_events(self._events)
            self._closed = True
