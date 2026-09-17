"""纯平面的导航定位健康观察，不拥有运动接口或定位提供方.

这里只比较调用方给出的位姿证据；不据不一致结果归责某个提供方，也不自动停车。
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Mapping, Optional


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class HealthPolicy:
    """导航侧健康告警策略，自动停车仍是未启用的 TODO-H1.

    Attributes:
        max_tf_age_sec: 位姿样本允许的最大历史年龄与未来偏差，单位 ROS 秒。
        max_divergence_m: 位移向量残差的严格超限阈值，单位米。
        divergence_window_sec: 配对观察的最长比较窗口，单位 ROS 秒。
        action: 只接受 warn；即使报告等级为 error 也不自动触发停车。
    """

    max_tf_age_sec: float = 0.5
    max_divergence_m: float = 1.0
    divergence_window_sec: float = 2.0
    action: str = 'warn'

    def __post_init__(self) -> None:
        """在分配观察资源前校验策略，不接受隐式开启停车行为.

        Raises:
            ValueError: 阈值不是正的有限 int/float，或 action 不是 warn。
            OverflowError: 超大整数无法转换为有限性检查所需的浮点数。
        """
        if any(not _finite(value) or value <= 0 for value in (
                self.max_tf_age_sec, self.max_divergence_m, self.divergence_window_sec)):
            raise ValueError('health thresholds must be positive finite numbers')
        if self.action != 'warn':
            raise ValueError('only warn is commissioned; automatic stop is TODO-H1')


@dataclass(frozen=True)
class PoseSample:
    """同一机器人参考点在某个坐标系中的一条平面位姿样本.

    Attributes:
        stamp: 数据源的 ROS 时钟时间戳，单位秒。
        x: 在该来源坐标系中的位置，单位米。
        y: 在该来源坐标系中的位置，单位米。
        yaw: 在该来源坐标系中的朝向，单位弧度；不在这里归一化。
    """

    stamp: float
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        """在样本进入历史或诊断前检查各字段为有限内置数值.

        不限制时间戳正负，也不对角度进行范围截取。

        Raises:
            ValueError: 字段不是有限 int/float，包括拒绝 bool。
            OverflowError: 超大整数无法进行浮点有限性检查。
        """
        if not all(_finite(value) for value in (self.stamp, self.x, self.y, self.yaw)):
            raise ValueError('pose sample values must be finite numbers')


@dataclass(frozen=True)
class HealthReport:
    """一份健康观察结果，不是停车请求或提供方故障归责.

    Attributes:
        severity: info、warn 或 error，与 NavEvent 等级约定一致。
        code: 稳定的事件或诊断状态名称。
        values: 数值证据映射；本数据类本身不验证有限性或复制、冻结映射内容，
            正常观察路径由 HealthMonitor 构造已检查的证据。
    """

    severity: str
    code: str
    values: Mapping[str, float]


class HealthMonitor:
    """在调用方单线程循环内保存有界配对历史并比较位移增量.

    Attributes:
        policy: 采样新鲜度、历史窗口和位移残差告警策略。
    """

    def __init__(self, policy: HealthPolicy) -> None:
        """保存策略并创建最多 4096 条的历史，不创建 ROS、时钟、线程或命令接口.

        Args:
            policy: 已校验的只告警策略。
        """
        self.policy = policy
        self._history = deque(maxlen=4096)
        self._last_now = None

    @property
    def history_size(self) -> int:
        """返回当前保留的配对观察数，最多 4096 条.

        Returns:
            当前历史长度，不修改或清理历史窗口。
        """
        return len(self._history)

    def sample(
        self, now: float, localized: bool, map_pose: Optional[PoseSample],
        wheel_pose: Optional[PoseSample],
    ) -> HealthReport:
        """比较新鲜配对位姿的位移增量，过期或时钟异常会重置比较历史.

        以窗口内最早一对的朝向差固定对齐轮式坐标轴，再比较两个位移向量，
        不比较绝对位置或标量路程。轮式位姿已由调用方积分并变换到同一导航参考点。
        两样本分别检查年龄，不要求源时间戳相等；历史按本次 now 计时。
        第一对也返回零残差的当前状态，尚不能据此声称已有完整窗口或一致性证据。

        Args:
            now: 当前 ROS 或仿真时间，单位秒，不是单调墙钟。
            localized: 配置的全局帧到里程计帧变换是否可用，必须是 bool。
            map_pose: 最近的全局帧到导航参考点位姿，无证据时为 None。
            wheel_pose: 同一参考点的最近轮式积分位姿，无证据时为 None。

        Returns:
            有限数值证据及按现有优先顺序可判定的健康状态；残差严格超过阈值
            才报告 error，不归责数据源也不请求停车。同一 now 会替换上一对样本。

        Raises:
            ValueError: 当前时间不是有限内置数值，或定位可用性不是 bool。
            OverflowError: 超大整数时间无法进行浮点有限性检查。
        """
        if not _finite(now) or type(localized) is not bool:
            raise ValueError('health time must be finite and localization state boolean')
        previous, self._last_now = self._last_now, now
        if previous is not None and now < previous:
            return self._reset('localization_clock_reset')
        if not localized:
            return self._reset('localization_waiting')
        evidence = {}
        for label, pose in (('tf', map_pose), ('odom', wheel_pose)):
            if pose is None:
                return self._reset('localization_' + label + '_missing', evidence)
            age = now - pose.stamp
            if not math.isfinite(age):
                return self._reset('localization_numeric_range', evidence)
            evidence[label + '_age_sec'] = age
            if age < -self.policy.max_tf_age_sec:
                return self._reset('localization_' + label + '_future', evidence)
            if age > self.policy.max_tf_age_sec:
                return self._reset('localization_' + label + '_stale', evidence)
        if previous is not None and now - previous > self.policy.divergence_window_sec:
            self._history.clear()
        cutoff = now - self.policy.divergence_window_sec
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()
        if self._history and self._history[-1][0] == now:
            self._history.pop()
        self._history.append((now, map_pose, wheel_pose))
        stamp, initial_map, initial_wheel = self._history[0]
        angle = math.remainder(initial_map.yaw, 2 * math.pi) - math.remainder(
            initial_wheel.yaw, 2 * math.pi)
        cosine, sine = math.cos(angle), math.sin(angle)
        dx, dy = wheel_pose.x - initial_wheel.x, wheel_pose.y - initial_wheel.y
        residual = math.hypot(map_pose.x - initial_map.x - (cosine * dx - sine * dy),
                              map_pose.y - initial_map.y - (sine * dx + cosine * dy))
        if not math.isfinite(residual):
            return self._reset('localization_numeric_range', evidence)
        evidence.update(divergence_m=residual, window_sec=now - stamp)
        if residual > self.policy.max_divergence_m:
            return HealthReport('error', 'localization_divergence', evidence)
        return HealthReport('info', 'localization_health_ok', evidence)

    def _reset(self, code, values=None):
        """清空健康观察历史并返回告警，不保留旧窗口继续判定.

        返回值仅供导航侧报告；不发送停机指令或修改定位状态。

        Args:
            code: 本次窗口重置的诊断代码。
            values: 可选诊断字段，复制为新的顶层字典。

        Returns:
            等级固定为 warn 的 HealthReport。
        """
        self._history.clear()
        return HealthReport('warn', code, dict(values or {}))
