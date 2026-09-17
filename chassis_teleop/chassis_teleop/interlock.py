"""在启动窗口内检查命令话题的发布者冲突，不管理其它 ROS 进程."""

import math
from typing import Callable, Sequence


class CommandConflict(RuntimeError):
    """现有发布者或无法可靠读取的发现结果导致拒绝键盘启动."""


def wait_for_clear_topics(
    topics: Sequence[str], query: Callable[[str], Sequence[object]],
    spin: Callable[[float], None], clock: Callable[[], float], duration_sec: float = 1.0,
) -> None:
    """在有限发现窗口内反复要求命令话题没有已发现的发布者.

    调用方应持有导航锁且尚未创建速度发布者；查询不是连续原子屏障，
    无法排除不遵守锁的进程随后启动，也不按节点名称豁免任一发布者。

    Args:
        topics: 已解析的绝对命令话题序列，包括键盘输出；这里只检查前导斜杠。
        query: 只读发布者信息查询，返回可取长度的序列；None 或查询异常拒绝启动。
        spin: 为图发现处理给定秒数内的回调，异常不包装为发布者冲突。
        clock: 返回单调时钟秒数的函数。
        duration_sec: 有限正发现时长，单位秒。

    Raises:
        ValueError: 话题列表或发现时长不合法。
        CommandConflict: 查询发现发布者，或发布者信息读取失败。
        Exception: 时钟或 spin 回调自身失败，直接传播给调用方。
    """
    if (not topics or isinstance(topics, str) or any(
            not isinstance(topic, str) or not topic.startswith('/') for topic in topics)):
        raise ValueError('guard requires resolved absolute command topics')
    if (isinstance(duration_sec, bool) or not isinstance(duration_sec, (int, float))
            or not math.isfinite(duration_sec) or duration_sec <= 0.0):
        raise ValueError('discovery window must be finite and positive')
    deadline = clock() + duration_sec
    while True:
        for topic in dict.fromkeys(topics):
            try:
                publishers = query(topic)
                if publishers is None:
                    raise RuntimeError('missing publisher observation')
                count = len(publishers)
            except Exception as error:
                message = f'publisher discovery failed for {topic}: {error}'
                raise CommandConflict(message) from error
            if count:
                raise CommandConflict(f'command topic {topic} already has {count} publisher(s)')
        remaining = deadline - clock()
        if remaining <= 0.0:
            return
        spin(min(remaining, 0.05))
