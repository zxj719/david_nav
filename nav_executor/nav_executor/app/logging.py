"""为进程生命周期发送队列提供冻结的发送前后审计文本."""

import json
from typing import Callable

from nav_executor.adapters.transport.logging import log_line


class OutboundAudit:
    """生成发送前错误日志及发送后的终态或重发日志，不证明远端接收."""

    def __init__(self, emit: Callable[[str], None] = log_line) -> None:
        """注入日志函数，不在构造时创建发送队列或传输连接.

        Args:
            emit: 接收无前缀文本的同步日志函数，默认使用既有日志器。
        """
        self._emit = emit

    def on_sending(self, encoded: str) -> None:
        """在每次尝试发送 on_error 前记录冻结日志，不表示发送已经成功.

        既有文本包含 reported to server，但此回调运行在 send 之前，
        不能据此判定服务端收到错误事件。日志异常会阻止本次 send 并保留待发帧。

        Args:
            encoded: ReliableOutbox.put 已冻结的 JSON 对象文本。

        Raises:
            ValueError: 传入文本不能按 JSON 解码。
            Exception: 日志函数或非预期负载处理失败，向队列发送者传播。
        """
        payload = json.loads(encoded)
        if payload.get('event_type') == 'on_error':
            self._emit(
                'NAV ERROR reported to server: '
                f"request_id={payload.get('request_id')} "
                f"sub_id={payload.get('sub_id')} "
                f"error_message={payload.get('error_message')}")

    def on_sent(self, encoded: str, resend: bool) -> None:
        """在本地 send 正常返回后记录终态或重发帧，普通进度不重复刷日志.

        发送队列已在调用前清除当前帧，因此日志异常不触发该帧重发。
        本地完成不是服务端业务确认，重发标志也不证明上次已传到远端。

        Args:
            encoded: 本地 send 已完成的 JSON 对象文本。
            resend: 先前 drain 是否为本帧保留过重发标志。

        Raises:
            ValueError: 传入文本不能按 JSON 解码。
            Exception: 日志函数或非预期负载处理失败。
        """
        payload = json.loads(encoded)
        event_type = payload.get('event_type') or payload.get('type')
        if resend or event_type in ('on_arrived', 'on_error'):
            tag = 'RESENT(断联重发)' if resend else 'sent'
            self._emit(
                f'OUT {tag}: {event_type} '
                f"request_id={payload.get('request_id')} sub_id={payload.get('sub_id')}")
