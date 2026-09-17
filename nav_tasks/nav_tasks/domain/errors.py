"""定义导航领域和任务准入异常，不依赖具体传输实现."""

from typing import Awaitable, Callable


class NavTaskError(Exception):
    """表示预期导航任务失败的公共异常基类."""


class PoseUnavailableError(NavTaskError):
    """表示当前无法取得所需位姿或坐标变换."""


class PoiNotFoundError(NavTaskError):
    """表示点位仓库中没有请求的 POI."""


class ConfigError(NavTaskError):
    """表示配置或准入参数未满足执行要求."""


class RuntimeAssemblyError(NavTaskError):
    """表示运行时组装失败且宿主仍需通过保留句柄重试清理.

    异常本身不会执行清理或销毁外部 ROS 节点；宿主须在原事件循环完成清理，
    成功前保留相关节点和资源。原始异常链由抛出端的 raise ... from 设置。

    Attributes:
        cleanup: 在原事件循环重试清理的异步回调；成功前资源仍归原宿主。
    """

    def __init__(
        self, cause: BaseException, cleanup: Callable[[], Awaitable[None]],
    ) -> None:
        """把原始失败写入异常文本，并保存供宿主重试的清理回调.

        本构造器不调用回调，也不自行设置 __cause__；运行时回滚调用方负责异常链。

        Args:
            cause: 原始构造失败，用于异常文本。
            cleanup: 在原事件循环重试未完成清理的异步回调。
        """
        super().__init__(f'runtime construction cleanup is incomplete: {cause}')
        self.cleanup = cleanup


class BusyError(NavTaskError):
    """表示现有任务仍持有运动所有权，新请求不能按当前策略抢占."""


class ClosedError(NavTaskError):
    """表示任务准入或后台工作入口已经关闭."""


class UnknownActionError(NavTaskError):
    """表示请求动作没有注册对应的解码器和处理器."""
