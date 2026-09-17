"""保留传输日志格式，不反向导入应用装配层."""


def log_line(message: str) -> None:
    """按既有执行器前缀打印一行并刷新标准输出.

    Args:
        message: 不含前缀或尾换行的完整消息；本函数不额外清洗内容。
    """
    print(f'[navigation_executor] {message}', flush=True)
