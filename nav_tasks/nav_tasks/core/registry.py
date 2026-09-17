"""提供可扩展的运动动作注册、同步解码和技能上下文，不导入线协议实现."""

import asyncio
from dataclasses import dataclass
from typing import Awaitable, Callable, Mapping

from nav_tasks.core.ports import FeedbackCallback
from nav_tasks.domain.errors import UnknownActionError
from nav_tasks.domain.tasks import NavOutcome


@dataclass(frozen=True)
class TaskContext:
    """保存技能关联身份和由管理器提供的任务循环回调.

    数据类本身不验证代次或回调线程；过期反馈屏蔽由注入的管理器回调实现。
    默认回调不更新状态，check_cancelled 只读取注入的取消谓词。

    Attributes:
        task_id: 上游任务标识。
        sub_id: 上游子任务编号。
        generation: 内部准入代次，区分重复使用的任务标识。
        feedback: 仅接受当前任务剩余距离的回调，单位米。
        cancel_requested: 管理器是否已经开始显式取消的查询函数。
        start_segment: 把新分段标为 STARTING，不发送到达事件。
        describe: 更新目标/阶段标签，不改变运动所有权。
    """

    task_id: str
    sub_id: int
    generation: int
    feedback: FeedbackCallback
    cancel_requested: Callable[[], bool] = lambda: False
    start_segment: Callable[[], None] = lambda: None
    describe: Callable[[str, str], None] = lambda target_id, phase: None

    def check_cancelled(self) -> None:
        """在开始新分段或重试前查询取消标记，不执行底层清理.

        Raises:
            asyncio.CancelledError: 注入的谓词报告任务已请求取消。
            Exception: 查询谓词自身失败。
        """
        if self.cancel_requested():
            raise asyncio.CancelledError


Decoder = Callable[[Mapping[str, object]], object]
Handler = Callable[[TaskContext, object], Awaitable[NavOutcome]]


class CommandRegistry:
    """保存动作解码器与异步技能映射，不拥有或抢占运动任务."""

    def __init__(self) -> None:
        """创建空注册表，不隐式注册运行时技能."""
        self._entries: dict[str, tuple[Decoder, Handler]] = {}

    def register(self, action: str, decoder: Decoder, handler: Handler) -> None:
        """登记一组动作处理函数，拒绝覆盖已有键.

        只检查动作键真值/重复和两个对象可调用；不检查空白名称、异步函数类别，
        也不证明解码器返回对象不可变，调用方必须遵守类型与技能契约。

        Args:
            action: 由入站选择的非空动作名称，应由调用方保证字符串语义。
            decoder: 同步纯解码器，按契约返回独立领域值。
            handler: 共享单一运动后端的异步技能函数。

        Raises:
            ValueError: 动作键为空或重复，或任一处理对象不可调用。
        """
        if not action or action in self._entries:
            raise ValueError(f'empty or duplicate task action: {action!r}')
        if not callable(decoder) or not callable(handler):
            raise ValueError('decoder and handler must be callable')
        self._entries[action] = (decoder, handler)

    def prepare(self, action: str, payload: Mapping[str, object]) -> tuple[Handler, object]:
        """同步解码请求后返回技能，供管理器在抢占前拒绝非法载荷.

        不执行技能、不深复制结果、不验证结果不可变性；解码器必须自行保证这些约束。

        Args:
            action: 已注册的动作键。
            payload: 与传输无关的入站字段映射。

        Returns:
            注册的技能函数和解码器原样返回的领域值。

        Raises:
            UnknownActionError: 动作键未注册。
            Exception: 解码器失败，异常原样传播。
        """
        if action not in self._entries:
            raise UnknownActionError(f'unknown task action: {action}')
        decoder, handler = self._entries[action]
        return handler, decoder(payload)
