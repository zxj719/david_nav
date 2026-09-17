"""为冻结的导航协议提供仅监听回环地址的脚本化 WebSocket 测试对端."""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from nav_sim.model import finite_number, require_sandbox


_CLI_DESCRIPTION = (
    'Loopback-only scripted WebSocket peer for the frozen navigation protocol.')


def navigate_to(request_id: str, target_id: str, sub_id: int = 0) -> Dict[str, Any]:
    """构造 v1 POI 命令并保持线上字段插入顺序，不在此发送.

    Args:
        request_id: 非空且非纯空白的任务字符串，保留原值。
        target_id: 非空且非纯空白的 POI 字符串，不接受直接坐标。
        sub_id: 整数子任务标识，拒绝布尔值。

    Returns:
        v1 navigate_to 命令字典。

    Raises:
        ValueError: 字符串标识不合法或 sub_id 不是非布尔整数。
    """
    if (not isinstance(request_id, str) or not request_id.strip()
            or not isinstance(target_id, str) or not target_id.strip()
            or isinstance(sub_id, bool) or not isinstance(sub_id, int)):
        raise ValueError('navigate_to requires request_id, target_id and integer sub_id')
    return {'action': 'navigate_to', 'request_id': request_id,
            'sub_id': sub_id, 'target_id': target_id}


def abort(request_id: str) -> Dict[str, str]:
    """构造既有 abort_navigation 取消命令，不引入新的动作别名或发送.

    Args:
        request_id: 非空且非纯空白的请求字符串，保留原值。

    Returns:
        v1 取消命令字典。

    Raises:
        ValueError: 请求标识不是字符串或为空、纯空白。
    """
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError('abort requires request_id')
    return {'action': 'abort_navigation', 'request_id': request_id}


@dataclass(frozen=True)
class ScriptStep:
    """记录一条待发命令及可选匹配事件屏障；冻结字段不等于深冻结映射.

    Attributes:
        command: 待发送的 v1 命令映射。
        wait_event: 发送下一条命令前等待的事件类型，None 表示无需等待。
    """

    command: Mapping[str, Any]
    wait_event: Optional[str] = None


def load_script(path: Path) -> List[ScriptStep]:
    """读取有序 JSON 场景，只允许导航和取消动作，不在此执行.

    Args:
        path: 内容为 command/wait_event 对象列表的 JSON 文件路径。

    Returns:
        按原顺序重建并校验的步骤列表；命令中未选用的额外字段不保留。
        文件和事件集合没有在此设置数量上限。

    Raises:
        ValueError: JSON 语法、脚本结构、动作或所需字段不合法。
        OSError: 脚本无法读取。
    """
    entries = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(entries, list):
        raise ValueError('scenario must be a list')
    result = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get('command'), dict):
            raise ValueError('scenario step requires command object')
        command = entry['command']
        action = command.get('action')
        if action == 'navigate_to':
            validated = navigate_to(command.get('request_id'), command.get('target_id'),
                                    command.get('sub_id', 0))
        elif action == 'abort_navigation':
            validated = abort(command.get('request_id'))
        else:
            raise ValueError(f'unsupported scenario action: {action}')
        event = entry.get('wait_event')
        if event is not None and (not isinstance(event, str) or not event):
            raise ValueError('wait_event must be a nonempty string or null')
        result.append(ScriptStep(validated, event))
    return result


@dataclass
class AgentSession:
    """驱动单连接的脚本步骤，可注入内存对端；接收事件列表没有容量限制.

    Attributes:
        steps: 待发命令及事件屏障序列。
        timeout: 每个事件屏障的超时，单位秒，不是整个会话的总时限。
        events: 按接收顺序保留的全部 JSON 对象事件。
        completed: 是否已走完脚本步骤和屏障，不是独立导航成功证据。
    """

    steps: Sequence[ScriptStep] = ()
    timeout: float = 30.0
    events: List[Dict[str, Any]] = field(default_factory=list)
    completed: bool = False

    async def run(self, websocket: Any) -> None:
        """依次发送脚本并保留交错事件，屏障结束后继续接收直到连接结束.

        Args:
            websocket: 实现异步 send、recv 和迭代的测试对端。

        Raises:
            ValueError: 超时不是有限正数，或对端帧不能解码为 JSON 对象。
            asyncio.TimeoutError: 在单个屏障期限内未收到关联匹配的事件。
        """
        if finite_number(self.timeout, 'timeout') <= 0:
            raise ValueError('timeout must be positive')
        for step in self.steps:
            await websocket.send(json.dumps(dict(step.command), ensure_ascii=False))
            if step.wait_event:
                await asyncio.wait_for(self._await_event(websocket, step), self.timeout)
        self.completed = True
        async for frame in websocket:
            await self._receive(websocket, frame)

    async def _await_event(self, websocket: Any, step: ScriptStep) -> None:
        """持续接收事件直到类型与请求身份匹配，保留期间无关帧.

        步骤含 sub_id 时还须完全匹配子任务 ID；不含时不额外限制。
        超时预算由外层 run 的 wait_for 负责，此处不自行判断导航位置。

        Args:
            websocket: 当前测试客户端的异步连接。
            step: 已下发命令及要等待的事件类型。

        Raises:
            asyncio.CancelledError: 外层等待超时或取消本任务。
            Exception: 连接接收、帧解码或心跳应答失败。
        """
        while True:
            event = await self._receive(websocket, await websocket.recv())
            if (event.get('event_type') == step.wait_event
                    and event.get('request_id') == step.command.get('request_id')
                    and ('sub_id' not in step.command
                         or event.get('sub_id') == step.command['sub_id'])):
                return

    async def _receive(self, websocket: Any, frame: str) -> Dict[str, Any]:
        """记录一帧 JSON 对象，并对应用层 ping 发送兼容 pong.

        事件先加入列表再尝试应答；这里不是 selftest_trace 的严格证据解码器，
        不额外检查重复键、结构深度或字段 schema。

        Args:
            websocket: 仅用于 ping 应答的当前异步连接。
            frame: 客户端发送的 JSON 文本。

        Returns:
            已追加到会话事件列表的对象字典。

        Raises:
            ValueError: 文本不是有效 JSON 或顶层不是对象。
            Exception: 心跳应答发送失败，接收事件仍已记录。
        """
        event = json.loads(frame)
        if not isinstance(event, dict):
            raise ValueError('agent peer must send JSON objects')
        self.events.append(event)
        if event.get('type') == 'ping':
            await websocket.send(json.dumps({'type': 'pong'}, ensure_ascii=False))
        return event


async def _serve(options: argparse.Namespace) -> None:
    """监听回环地址并串行接收自测客户端，不自行授予脚本目标执行权限.

    Args:
        options: 已经由 main 校验的端口、脚本、秒制超时和可选事件输出路径；
            有脚本时调用方必须已检查 --allow-goals。事件文件采用追加写入。

    Raises:
        OSError: 脚本读取或监听端口失败；会话事件写入异常由服务回调报告。
        ValueError: 脚本内容不符合命令步骤格式。
        asyncio.CancelledError: 外层终止服务，异步上下文负责关闭监听资源。
    """
    from websockets.legacy.server import serve

    steps = load_script(options.script) if options.script else []
    active = False

    async def handler(websocket):
        """串行运行一个测试会话，退出时追加已接收事件并释放接入标志.

        已有活动会话时以 1013 拒绝新连接。会话运行结束或失败都会先清除
        active，再同步追加事件；写文件失败会传播，也可能替代原会话异常。
        不在此重新检查运动授权，脚本执行授权必须由外层入口先确认。

        Args:
            websocket: 服务交付的单个异步连接，或测试用内存替身。

        Raises:
            asyncio.CancelledError: 服务或外层取消会话，finally 仍尝试记录。
            Exception: 连接、脚本等待、事件追加或退出打印失败。
        """
        nonlocal active
        if active:
            await websocket.close(code=1013, reason='one test client at a time')
            return
        active = True
        session = AgentSession(steps, options.timeout)
        try:
            await session.run(websocket)
        finally:
            active = False
            if options.events:
                with options.events.open('a', encoding='utf-8') as stream:
                    for event in session.events:
                        stream.write(json.dumps(event, ensure_ascii=False) + '\n')
            print(f'client closed: completed={session.completed} events={len(session.events)}',
                  flush=True)

    async with serve(handler, '127.0.0.1', options.port):
        print(f'fake agent listening on 127.0.0.1:{options.port}', flush=True)
        await asyncio.Future()


def main(args: Optional[Sequence[str]] = None) -> None:
    """运行本地被动对端；脚本发送另需显式开关，帮助路径不启动服务.

    域 99 和 allow-goals 是入口检查，不能代替现场运动授权。
    参数解析在域检查之前；正常运行只监听 127.0.0.1，捕获键盘中断。

    Args:
        args: 可选命令行参数；None 时读取进程命令行。

    Raises:
        RuntimeError: 未显式选择域 99。
        SystemExit: 显示帮助，或参数、脚本授权、端口及超时检查未通过。
    """
    parser = argparse.ArgumentParser(description=_CLI_DESCRIPTION)
    parser.add_argument('--port', type=int, default=18101)
    parser.add_argument('--script', type=Path)
    parser.add_argument('--allow-goals', action='store_true')
    parser.add_argument('--events', type=Path)
    parser.add_argument('--timeout', type=float, default=30.0)
    options = parser.parse_args(args)
    require_sandbox(os.environ)
    if options.script and not options.allow_goals:
        parser.error('--script requires explicit --allow-goals; default server is passive')
    if not 1 <= options.port <= 65535:
        parser.error('--port must be in 1..65535')
    try:
        if finite_number(options.timeout, 'timeout') <= 0:
            parser.error('--timeout must be positive')
    except ValueError as error:
        parser.error(str(error))
    try:
        asyncio.run(_serve(options))
    except KeyboardInterrupt:
        pass
