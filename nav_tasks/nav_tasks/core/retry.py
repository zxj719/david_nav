"""实现与底层目标下发分离的单任务失败重试策略."""

import asyncio
from dataclasses import dataclass
import math
import time
from typing import Awaitable, Callable

from nav_tasks.core.ports import Clock
from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.tasks import NavOutcome, NavStatus


class MonotonicClock:
    """提供间隔计时用单调时钟，不能用来比较来源位姿时间戳."""

    def now(self) -> float:
        """读取单调时钟秒数.

        Returns:
            time.monotonic 的当前值，不是墙钟时间戳。
        """
        return time.monotonic()

    async def sleep(self, seconds: float) -> None:
        """通过 asyncio.sleep 让出循环等待，不阻塞事件线程.

        Args:
            seconds: 调用方提供的等待秒数，通常应非负；本方法不另行校验范围。

        Raises:
            asyncio.CancelledError: 等待被取消。
        """
        await asyncio.sleep(seconds)


@dataclass(frozen=True)
class RetryFailure:
    """保存每次 FAILED 后的只读观察值，在延迟或耗尽返回前发出.

    Attributes:
        attempt: 本次已失败的尝试序号，从 1 开始。
        delay_sec: 配置的重试间隔（秒），不表示一定还会重试。
        detail: 本次失败的后端说明。
        total_attempts: 包含首次尝试的总预算。
    """

    attempt: int
    total_attempts: int
    delay_sec: float
    detail: str


@dataclass(frozen=True)
class RetryPolicy:
    """只对 FAILED 结果重试，不把协程取消或后端异常改写为导航失败.

    retry_count 是首次之后的额外次数；取消结果直接返回，不重试。
    该策略不验证结果对象类型，尝试函数须遵守 NavOutcome 契约。

    Attributes:
        retry_count: FAILED 后允许的额外尝试次数。
        retry_delay_sec: 两次尝试之间的等待时间（秒）。
    """

    retry_count: int = 1
    retry_delay_sec: float = 1.0

    def __post_init__(self) -> None:
        """在首次尝试前验证重试次数与延迟预算.

        次数要求非负且类型恰为 int；延迟只做有限性和非负检查，不执行严格类型门禁。

        Raises:
            ConfigError: 次数或受检延迟范围不合法。
            TypeError: 延迟不能参与有限性检查。
            OverflowError: 延迟数值过大，不能转换为有限性检查所需的浮点值。
        """
        if type(self.retry_count) is not int or self.retry_count < 0:
            raise ConfigError('retry_count must be a nonnegative integer')
        if not math.isfinite(self.retry_delay_sec) or self.retry_delay_sec < 0.0:
            raise ConfigError('retry_delay_sec must be finite and nonnegative')

    async def run(
        self, attempt: Callable[[], Awaitable[NavOutcome]], clock: Clock,
        on_failure: Callable[[RetryFailure], None] | None = None,
    ) -> NavOutcome:
        """顺序执行尝试，保持既有耗尽错误文本和次数语义.

        每次 FAILED 都先通知同步观察者，含最后一次；仅仍有预算时等待再试。
        不捕获尝试、观察者或时钟错误，也不代替后端取消清理。

        Args:
            attempt: 每次创建全新单次尝试协程的工厂。
            clock: 可注入的间隔时钟。
            on_failure: 可选同步失败观察者，异常原样传播并中止后续重试。

        Returns:
            第一个非 FAILED 结果，或附旧格式总尝试次数说明的 FAILED。

        Raises:
            asyncio.CancelledError: 当前尝试或等待间隔被取消。
            Exception: 尝试、观察者或时钟失败。
        """
        total = self.retry_count + 1
        for index in range(total):
            outcome = await attempt()
            if outcome.status != NavStatus.FAILED:
                return outcome
            if on_failure is not None:
                on_failure(RetryFailure(index + 1, total, self.retry_delay_sec, outcome.detail))
            if index + 1 < total:
                await clock.sleep(self.retry_delay_sec)
        return NavOutcome(NavStatus.FAILED, f'导航失败(共尝试{total}次): {outcome.detail}')
