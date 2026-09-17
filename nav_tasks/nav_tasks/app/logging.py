"""输出稳定的重试审计文本，与纯重试策略及传输实现分离."""

from typing import Callable

from nav_tasks.core.registry import TaskContext
from nav_tasks.core.retry import RetryFailure


def _print_line(line: str) -> None:
    print(line, flush=True)


class RetryAudit:
    """通过注入日志接收器输出旧 bridge 重试审计，不负责重试或消息发送."""

    def __init__(self, emit: Callable[[str], None] = _print_line) -> None:
        """保存同步整行日志接收器，不启动后台任务.

        Args:
            emit: 同步整行输出函数，默认使用立即 flush 的标准输出。
        """
        self._emit = emit

    def __call__(self, context: TaskContext, failure: RetryFailure) -> None:
        """输出一次失败及其重试或耗尽决定，保留冻结文本.

        只观察传入的次数与预算；日志提及重发或上报不证明对应副作用已经完成。
        接收器或字段格式化异常原样传播，第一行已输出时不会回滚它。

        Args:
            context: 用于日志关联的任务及子任务身份。
            failure: 重试策略观察值；本观察器不修改该策略。
        """
        attempt, total = failure.attempt, failure.total_attempts
        self._emit(
            f'[nav2_action_bridge] NAV FAILURE attempt {attempt}/{total} '
            f'request_id={context.task_id} sub_id={context.sub_id}: {failure.detail}')
        if attempt < total:
            self._emit(
                '[nav2_action_bridge] suppressing failure report, re-publishing goal in '
                f'{failure.delay_sec:.1f}s (attempt {attempt + 1}/{total})')
        else:
            self._emit(
                f'[nav2_action_bridge] retry budget exhausted ({total} attempts) — '
                f'reporting on_error to server: {failure.detail}')
