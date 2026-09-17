"""保持原适配器限幅、非零速度提升和单次超时停车语义的纯计算模块.

不导入 ROS、不读取时钟、不发布命令；输入时刻由调用方提供。
速度整形不等于硬件已执行，也不在这里验证实体是否停稳。
"""

from dataclasses import asdict, dataclass
import math
from typing import Dict, Optional, Tuple, Union


@dataclass(frozen=True)
class AdapterConfig:
    """底盘适配器参数，档案配置可覆盖兼容旧入口的独立启动默认值.

    线速度采用米每秒，角速度采用弧度每秒；最小原地旋转速度用于提升命令，
    不是 Dock 的停稳判定阈值。

    Attributes:
        input_cmd_vel_topic: 由模式选择的速度输入话题。
        output_cmd_vel_topic: 整形后速度的输出话题。
        cmd_vel_timeout: 以 ROS 时间计的输入静默期限，单位秒；非正值禁用。
        min_linear_x: 非零前进或后退命令的最小幅值。
        max_linear_x: 前向速度幅值上限。
        min_angular_z: 行进中或未达到原地旋转选择阈值时的角速度最小幅值。
        min_stationary_angular_z: 有意原地旋转时的角速度最小幅值。
        stationary_angular_threshold: 用原始角速度幅值选择原地旋转提升的阈值。
        max_angular_z: 角速度幅值上限。
        publish_hz: 请求的定时发布频率，计算周期时按旧逻辑至少取 1 Hz。
    """

    input_cmd_vel_topic: str = 'cmd_vel_safe'
    output_cmd_vel_topic: str = '/cmd_vel'
    cmd_vel_timeout: float = 2.0
    min_linear_x: float = 0.20
    max_linear_x: float = 0.20
    min_angular_z: float = 0.0
    min_stationary_angular_z: float = 0.35
    stationary_angular_threshold: float = 0.025
    max_angular_z: float = 0.50
    publish_hz: float = 50.0

    def validate(self) -> None:
        """在创建 ROS 通信端点前拒绝空话题、无效数值和负速度限制.

        bool 不视为数值；超时和频率允许非正有限值，分别沿用禁用与最低频率逻辑。
        此处不校验话题完整 ROS 语法或上下限顺序，整形时会把最小幅值限制在上限内。

        Raises:
            ValueError: 话题不是非空字符串，数值类型错误或非有限，或速度限制为负。
        """
        for key, value in asdict(self).items():
            if key.endswith('_topic'):
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f'{key} must be a nonempty topic')
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value)):
                raise ValueError(f'{key} must be a finite number')
            elif key not in ('cmd_vel_timeout', 'publish_hz') and value < 0:
                raise ValueError(f'{key} must be nonnegative')

    @property
    def period_sec(self) -> float:
        """返回秒制定时周期，请求频率按旧逻辑至少取 1 Hz.

        Returns:
            1 除以 publish_hz 与 1.0 的较大值；调用前应完成数值校验。
        """
        return 1.0 / max(self.publish_hz, 1.0)


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return max(min(float(value), limit), -limit)


def _lift(value: float, minimum: float) -> float:
    """把非零小幅命令提升到最小幅值，保留方向且零始终为零.

    本助手不执行上限裁剪；调用方负责先限幅，并保证最小值不超过上限。
    输入有限性由调用方负责，不在此验证。

    Args:
        value: 已限幅的线速度或角速度，与 minimum 使用相同单位。
        minimum: 最小非零幅值，使用其绝对值；零表示不提升。

    Returns:
        保留原方向的浮点命令，大于等于最小幅值时保持原值。
    """
    value = float(value)
    minimum = abs(float(minimum))
    if value == 0.0 or minimum <= 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum


def shape_command(
    linear_x: float, linear_y: float, angular_z: float, config: AdapterConfig,
) -> Tuple[float, float]:
    """沿用先限幅后提升非零速度、按原始输入选择原地旋转阈值的逻辑.

    零始终不提升；横向输入不输出，但会影响是否选择原地旋转的最小角速度。
    最小幅值先约束到相应上限内，本函数不新增输入速度有限性检查。

    Args:
        linear_x: 原始前向命令，单位米每秒。
        linear_y: 原始横向输入，单位米每秒，仅用于选择角速度最小幅值。
        angular_z: 原始偏航角速度命令，单位弧度每秒。
        config: 已校验的适配器参数。

    Returns:
        整形后的前向速度和偏航角速度二元组；ROS 包装层的其它 Twist 轴保持零。
    """
    minimum = config.min_angular_z
    if angular_z == 0.0:
        minimum = 0.0
    elif not (abs(linear_x) > 0.0 or abs(linear_y) > 0.0):
        if abs(angular_z) >= abs(config.stationary_angular_threshold):
            minimum = config.min_stationary_angular_z
    return (
        _lift(_clamp(linear_x, config.max_linear_x),
              min(config.min_linear_x, config.max_linear_x)),
        _lift(_clamp(angular_z, config.max_angular_z), min(minimum, config.max_angular_z)))


class AdapterState:
    """保存最新命令及单次超时转移状态，不持有时钟或输入输出资源.

    Attributes:
        config: 已校验的速度整形和超时参数。
        input_count: 已接收输入命令的累计次数。
        last_input: 最后一次输入的 ROS 时间（整数纳秒）；初始设为一小时前。
        latest: 最近整形后的前向速度（米每秒）和偏航速度（弧度每秒）。
    """

    def __init__(self, config: AdapterConfig, now: int) -> None:
        """初始化零命令和兼容旧行为的输入时间、超时发送标记.

        初始 last_input 比 now 早一小时，停车已发送标记为真；
        是否实际判为过期仍取决于配置的超时值。

        Args:
            config: 将在此再次校验的适配器参数。
            now: 初始 ROS 时间，单位整数纳秒。

        Raises:
            ValueError: 适配器配置校验失败。
        """
        config.validate()
        self.config = config
        self.latest = (0.0, 0.0)
        self.last_input = now - 3600_000_000_000
        self.input_count = 0
        self._stale_stop_sent = True

    def receive(self, x: float, y: float, yaw: float, now: int) -> None:
        """保存一条整形后的命令，记录接收时间并复位超时发送标记.

        Args:
            x: 原始前向速度，单位米每秒。
            y: 原始横向速度，单位米每秒，仅参与旧有角速度最小值选择。
            yaw: 原始偏航角速度，单位弧度每秒。
            now: 接收命令时的 ROS 时间，单位整数纳秒。
        """
        self.latest = shape_command(x, y, yaw, self.config)
        self.last_input = now
        self.input_count += 1
        self._stale_stop_sent = False

    def is_stale(self, now: int) -> bool:
        """按 ROS 时间严格大于超时值的条件判断输入是否过期.

        不使用墙钟，也不单独处理 ROS 时间回退。

        Args:
            now: 当前 ROS 时间，单位整数纳秒。

        Returns:
            超时被禁用时为 False；否则仅在输入年龄严格超过期限时为 True。
        """
        return (self.config.cmd_vel_timeout > 0.0
                and (now - self.last_input) / 1e9 > self.config.cmd_vel_timeout)

    def output(self, now: int) -> Optional[Tuple[float, float]]:
        """取得本次定时器应发布的命令，并消费尚未发送的超时停车状态.

        Args:
            now: 当前 ROS 时间，单位整数纳秒。

        Returns:
            未过期时为最新命令；新一次过期只返回一次零速，之后为 None 表示静默。
            初始停车标记已置位，初始输入若过期则直接静默；不证明实体已停止。
        """
        if self.is_stale(now):
            if not self._stale_stop_sent:
                self._stale_stop_sent = True
                return (0.0, 0.0)
            return None
        return self.latest

    def diagnostics(self, now: int) -> Dict[str, Union[bool, int, float]]:
        """给出限幅参数和当前命令诊断，不消费超时状态转移.

        Args:
            now: 当前 ROS 时间，单位整数纳秒。

        Returns:
            输入计数、过期标记、输出速度和限制参数的字典；过期时速度字段为零，
            不代表刚刚发布了零速或硬件已经执行。
        """
        stale = self.is_stale(now)
        x, yaw = (0.0, 0.0) if stale else self.latest
        return {
            'input_count': self.input_count, 'timed_out': stale,
            'output_linear_x': x, 'output_angular_z': yaw,
            'max_linear_x': self.config.max_linear_x,
            'max_angular_z': self.config.max_angular_z,
            'min_linear_x': self.config.min_linear_x,
            'min_angular_z': self.config.min_angular_z,
            'min_stationary_angular_z': self.config.min_stationary_angular_z,
        }
