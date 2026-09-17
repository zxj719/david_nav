"""定义同步 Dock 工作线程端口，不依赖 ROS 或具体传输实现."""

from dataclasses import dataclass
from typing import Protocol

PlanarPose = tuple[float, float, float]


@dataclass(frozen=True)
class DockActionResult:
    """记录单段动作结果及是否因显式任务取消而返回.

    失败不表示可以放弃运动所有权；实现桥即使遇到期限也必须保留清理责任。
    canceled 表示显式任务停止或桥接协程取消，不是 Nav2 外部 CANCELED 状态。
    后者保留旧的单段失败语义，阶段二仍可能转入实测判定。本记录不证明实体停稳。

    Attributes:
        canceled: 是否由显式任务停止或桥接取消导致返回，默认 False。
        succeeded: 单段后端是否报告成功，不代表整个 Dock 已到达。
    """

    succeeded: bool
    canceled: bool = False


class DockMotionPort(Protocol):
    """提供唯一共享运动后端的阻塞线程视图，不创建第二个运动所有者.

    仅供受管工作线程调用；实现桥负责动作等待、冻结审计日志与清理责任。
    协议本身不实现线程安全、硬超时或资源回收。
    """

    def navigate(
        self, pose: PlanarPose, label: str, timeout_sec: float, extra_distance: float,
    ) -> DockActionResult:
        """通过共享后端执行一段导航并输出冻结动作审计行.

        Args:
            pose: 配置全局坐标系中的导航基座 x/y/yaw，单位米及弧度。
            label: 用于日志的冻结阶段名称。
            timeout_sec: 动作结果等待期限（秒），与接收及清理期限分开。
            extra_distance: 转发反馈时额外计入的距离（米）。

        Returns:
            单段成功与显式取消标志；返回失败不解除实现桥的清理责任。
            结果期限并不是包含阻塞桥与清理在内的硬墙钟上限。
        """
        ...

    def spin(
        self, yaw: float, label: str, time_allowance_sec: float, timeout_sec: float,
    ) -> DockActionResult:
        """通过与导航相同的共享后端执行相对偏航旋转.

        Args:
            yaw: 有符号相对偏航角（弧度）。
            label: 用于日志的冻结补偿阶段名称。
            time_allowance_sec: 交给服务端的执行预算（秒）。
            timeout_sec: 本地结果等待期限（秒），不包含独立清理期限。

        Returns:
            旋转成功与显式取消标志；成功本身不证明停稳或最终到达。
            实现桥仍需保留未完成的清理责任。
        """
        ...


class DockLogger(Protocol):
    """约定注入日志器提供既有 ROS 同名方法并可在线程间安全使用.

    线程安全由实现者保证；此协议不加锁，不改变冻结日志内容或等级。
    """

    def info(self, text: str) -> None:
        """以信息等级记录完整冻结日志行.

        Args:
            text: 已含 dock 前缀的完整日志文本。
        """
        ...

    def warn(self, text: str) -> None:
        """以警告等级记录完整冻结日志行.

        Args:
            text: 已含 dock 前缀的完整日志文本。
        """
        ...

    def error(self, text: str) -> None:
        """以错误等级记录完整冻结日志行.

        Args:
            text: 已含 dock 前缀的完整日志文本。
        """
        ...
