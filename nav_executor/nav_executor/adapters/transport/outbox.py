"""在进程存活期间有序缓存待发送帧，跨连接保留未完成的发送."""

import asyncio
import json
from typing import Awaitable, Callable


class ReliableOutbox:
    """保留已序列化帧直到本地 send 正常返回，不等待服务端业务确认.

    须在同一任务事件循环中构造和使用。本对象不创建连接或后台任务；
    运行器为连接持有 drain，重连复用 outbox 而非失败的 drain。
    发送结果不明时可能重发，因此不保证恰好一次交付。队列仅驻内存且无容量上限，
    不保证进程重启后恢复，也不证明服务端已收到或处理。
    """

    def __init__(
        self, *, on_sending: Callable[[str], None] | None = None,
        on_sent: Callable[[str, bool], None] | None = None,
    ) -> None:
        """在当前事件循环创建空发送队列和可选审计回调.

        Args:
            on_sending: 每次尝试 send 前接收已编码帧；异常会阻止本次发送。
            on_sent: send 正常返回并移除待发帧后接收帧和重发标志；
                此回调失败不将帧放回队列。

        Raises:
            RuntimeError: 当前线程没有运行中的事件循环。
        """
        self._loop = asyncio.get_running_loop()
        self._queue: asyncio.Queue[str] = asyncio.Queue()
        self._pending: str | None = None
        self._resend = False
        self._drain_lock = asyncio.Lock()
        self._on_sending = on_sending
        self._on_sent = on_sent

    @property
    def in_flight_count(self) -> int:
        """返回保留中的单个发送槽占用数（0 或 1），供既有重连日志使用."""
        self._check_loop()
        return int(self._pending is not None)

    @property
    def pending_count(self) -> int:
        """返回队列长度加当前发送槽占用数，不代表服务端未确认消息数."""
        self._check_loop()
        return self._queue.qsize() + int(self._pending is not None)

    def put(self, frame: dict) -> None:
        """按既有 JSON 格式立即序列化后入队，随后修改原对象不影响帧.

        保持键插入顺序与 Unicode；沿用 json.dumps 默认的非有限数策略，
        不是严格 schema 校验或持久存储。

        Args:
            frame: 有序 WS 字典，由上层负责业务字段合法性。

        Raises:
            RuntimeError: 在其它或未运行的事件循环中使用。
            TypeError: 内容无法 JSON 序列化，不入队。
            ValueError: 序列化检测到循环引用，不入队。
        """
        self._check_loop()
        self._queue.put_nowait(json.dumps(frame, ensure_ascii=False))

    async def drain(self, send: Callable[[str], Awaitable[None]]) -> None:
        """有序发送直到取消或回调失败，用锁保证同一时刻只有一个发送者.

        失败或被取消的 send 保留当前槽，优先于后续新帧。
        on_sending 失败也保留并标记下次重发；send 正常返回后先清槽再调用 on_sent，
        因此发送后日志失败不会重发。这里只依据本地 send 返回，不检查远端 ACK。

        Args:
            send: 异步传输发送操作，通常为 websocket.send。

        Raises:
            RuntimeError: 在非所属事件循环使用。
            asyncio.CancelledError: 调用方取消；已有的未完成发送槽仍保留。
            Exception: 发送或审计回调失败，调用方可在重连后再次 drain。
        """
        self._check_loop()
        async with self._drain_lock:
            while True:
                if self._pending is None:
                    self._pending = await self._queue.get()
                    self._resend = False
                encoded = self._pending
                resend = self._resend
                self._resend = True
                if self._on_sending is not None:
                    self._on_sending(encoded)
                await send(encoded)
                self._pending = None
                self._resend = False
                if self._on_sent is not None:
                    self._on_sent(encoded, resend)

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('outbox used from a different event loop')
