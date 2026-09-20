"""同用户之间的协作式控制，由既有的运行观察线程轮询.

使用 Linux 抽象命名空间的顺序包套接字，不需要套接字文件、守护进程或额外线程。
端点按每次启动重新生成，因此上一次运行的旧客户端停不掉新的运行。
发现信息与对端 PID 只用于识别，任何路径都不会据此发送操作系统信号；
"协作式"意味着只有愿意来问的客户端会被影响，不能用它强制终止任何进程。
"""

from __future__ import annotations

import json
import os
import socket
import struct
import time
from typing import Any, Callable, Dict
import uuid

from nav_bringup.navrun.locking import RunLock


class RunControl:
    """只在创建时的租约仍被持有期间，提供有界的状态查询与停止请求服务.

    每次 poll 最多处理八个请求，且只服务同一用户；不新建线程，
    也不保证请求一定被及时处理——推进完全取决于调用方的轮询频率。
    """

    def __init__(self, lease: RunLock, request_stop: Callable[[], None]) -> None:
        """只描述一个协作式服务端，不打开任何端点.

        构造阶段不绑定套接字、不写发现信息，也不核对租约归属。

        Args:
            lease: 已取得的租约，直到收尾都由调用方持有。
            request_stop: 幂等的停止请求，不是进程信号，也不代表清理已完成。
        """
        self._lease = lease
        self._request_stop = request_stop
        self._socket = None
        self._clients: Dict[socket.socket, float] = {}
        instance = uuid.uuid4().hex
        self._metadata: Dict[str, Any] = {
            'version': 1, 'instance': instance, 'address': f'navrun_{os.geteuid()}_{instance}',
            'run_id': None, 'mode': None, 'finished': False}
        self._stopped = False

    def __enter__(self) -> RunControl:
        """打开本次启动专属的端点，并把发现信息写进租约注记.

        绑定失败或写注记失败时会关闭刚建的监听套接字再外抛，不留下半开状态。

        Returns:
            已激活的服务端；调用方必须自行安排周期性的 poll 调用，
            否则端点存在但没有人应答。

        Raises:
            RuntimeError: 服务端已打开，或租约不归本进程所有。
            OSError: 套接字创建或发现信息写入失败。
        """
        if self._socket is not None:
            raise RuntimeError('control server already open')
        self._lease.assert_owned()
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            listener.setblocking(False)
            listener.bind('\0' + self._metadata['address'])
            listener.listen(8)
            self._lease.annotate(self._metadata)
        except BaseException:
            listener.close()
            raise
        self._socket = listener
        return self

    def identify(self, run_id: str, mode: str) -> None:
        """在观察线程启动前公布新的运行记录身份.

        只更新注记内容，不影响已建立的连接，也不校验该记录目录是否存在。

        Args:
            run_id: 运行记录名，仅作标识，不是文件系统上的操作对象。
            mode: 选定的导航或底盘模式。
        """
        self._metadata.update(run_id=run_id, mode=mode)
        self._lease.annotate(self._metadata)

    def poll(self) -> None:
        """每轮观察最多处理八个非阻塞的同用户请求.

        畸形、超长、过期或过慢的客户端一律直接关闭，不触发停止流程。
        端点未打开时直接返回，不报错。只应由调用方既有的观察线程调用，
        本方法自身不加锁，并发调用没有保证。
        """
        if self._socket is None:
            return
        for _ in range(8 - len(self._clients)):
            try:
                client, _ = self._socket.accept()
            except BlockingIOError:
                break
            client.setblocking(False)
            _, uid, _ = struct.unpack('3i', client.getsockopt(
                socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i')))
            if uid != os.geteuid():
                client.close()
            else:
                self._clients[client] = time.monotonic() + 2.0
        for client, deadline in list(self._clients.items()):
            try:
                if time.monotonic() >= deadline:
                    self._close_client(client)
                    continue
                packet, _, flags, _ = client.recvmsg(1024)
                if flags & socket.MSG_TRUNC or not packet:
                    self._close_client(client)
                    continue
                request = json.loads(packet)
                if (not isinstance(request, dict) or set(request) != {'op', 'instance'}
                        or request['instance'] != self._metadata['instance']
                        or request['op'] not in ('status', 'down')):
                    self._close_client(client)
                    continue
                self._lease.assert_owned()
                if request['op'] == 'down' and not self._stopped:
                    self._request_stop()
                    self._stopped = True
                response = dict(self._metadata, pid=os.getpid(),
                                state='stop_requested' if self._stopped else 'active',
                                cleanup_complete=False)
                client.sendall(json.dumps(response).encode('utf-8'))
                self._close_client(client)
            except BlockingIOError:
                continue
            except (ValueError, OSError, RuntimeError):
                self._close_client(client)

    def _close_client(self, client: socket.socket) -> None:
        self._clients.pop(client, None)
        client.close()

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """先关闭控制端点再释放租约；只有正常收尾才标记完成.

        调用方必须在离开本上下文之前 join 自己的轮询线程，否则关闭中的套接字
        可能仍被轮询。带异常离开时不写 finished 标记，因此外部读到的
        "未完成"只说明没有证明正常收尾，不说明进程一定还活着。

        Args:
            exc_type: 逸出的异常类型；完成清理后为 None。
            exc_value: 逸出的异常实例，未使用。
            traceback: 逸出的异常回溯，未使用。
        """
        for client in list(self._clients):
            self._close_client(client)
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if exc_type is None:
            self._metadata['finished'] = True
            self._lease.annotate(self._metadata)
