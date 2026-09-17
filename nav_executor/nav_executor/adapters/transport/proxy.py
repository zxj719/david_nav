"""提供显式调整当前进程代理绕过列表的辅助函数，不在导入时改环境."""

import os
from urllib.parse import urlsplit


def bypass_proxy_for_server(server_uri: str) -> None:
    """沿用旧策略，将配置主机追加到两个代理绕过环境变量.

    只修改当前进程的 no_proxy/NO_PROXY，不更改系统代理文件。
    按精确字符串判断是否已包含；若已包含，不重写该变量的原有格式。

    Args:
        server_uri: 取 hostname 加入绕过列表的 WS URI；没有主机名时不修改。

    Raises:
        ValueError: URI 主机部分无法由 urlsplit 解析。
    """
    host = urlsplit(server_uri).hostname
    if not host:
        return
    for key in ('no_proxy', 'NO_PROXY'):
        entries = [entry.strip() for entry in os.environ.get(key, '').split(',') if entry.strip()]
        if host not in entries:
            entries.append(host)
            os.environ[key] = ','.join(entries)
