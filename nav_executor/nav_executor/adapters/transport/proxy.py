"""Keep the configured local agent host out of environment-driven proxies."""

import os
from urllib.parse import urlsplit


def bypass_proxy_for_server(server_uri: str) -> None:
    """Append the configured host to both proxy bypass variables, as before.

    Only explicit callers change the process environment; imports have no effect.

    Args:
        server_uri: Agent WebSocket URI whose hostname should bypass the proxy.
    """
    host = urlsplit(server_uri).hostname
    if not host:
        return
    for key in ('no_proxy', 'NO_PROXY'):
        entries = [entry.strip() for entry in os.environ.get(key, '').split(',') if entry.strip()]
        if host not in entries:
            entries.append(host)
            os.environ[key] = ','.join(entries)
