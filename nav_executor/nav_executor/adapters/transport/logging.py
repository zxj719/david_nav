"""Preserve the transport logger without importing application assembly."""


def log_line(message: str) -> None:
    """Write a flushed line with the legacy execution logger prefix.

    Args:
        message: Complete line without prefix or newline.
    """
    print(f'[navigation_executor] {message}', flush=True)
