"""保持旧键盘操作语义的纯命令状态，不执行 ROS 或终端操作.

时钟值由调用方提供，控制逻辑只返回前向和偏航意图，不生成横向速度。
"""

from dataclasses import asdict, dataclass
import math
from typing import Optional, Tuple


_ARROWS = {'\x1b[A': (1.0, 0.0), '\x1b[B': (-1.0, 0.0),
           '\x1b[D': (0.0, 1.0), '\x1b[C': (0.0, -1.0)}
_WASD = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'a': (0.0, 1.0), 'd': (0.0, -1.0)}
_UIOJKL = {'u': (1.0, 1.0), 'i': (1.0, 0.0), 'o': (1.0, -1.0),
           'j': (0.0, 1.0), 'l': (0.0, -1.0), 'm': (-1.0, -1.0),
           ',': (-1.0, 0.0), '.': (-1.0, 1.0)}


@dataclass(frozen=True)
class TeleopConfig:
    """遥控启动参数，保留旧独立入口的默认速度和终端选择.

    Attributes:
        cmd_vel_topic: 遥控输出话题，接入适配器而非直接驱动；实际接线由配置负责。
        linear_speed: 当前前进或后退档位幅值，单位米每秒。
        angular_speed: 当前偏航档位幅值，单位弧度每秒。
        linear_deadzone: 最小非零线速度命令，单位米每秒。
        angular_deadzone: 最小非零角速度命令，单位弧度每秒。
        max_linear_x: 键盘线速度幅值上限，单位米每秒。
        max_angular_z: 键盘角速度幅值上限，单位弧度每秒。
        publish_hz: 主循环频率，计算周期时至少取 1 Hz。
        key_timeout: 基于单调时钟的按键失活期限，单位秒；非正值禁用。
        layout: wasd 或 uiojkl；两者均支持方向键。
        tty_device: 显式终端设备路径，不依赖重定向后的标准输入。
    """

    cmd_vel_topic: str = 'cmd_vel_safe'
    linear_speed: float = 0.20
    angular_speed: float = 0.35
    linear_deadzone: float = 0.20
    angular_deadzone: float = 0.35
    max_linear_x: float = 0.20
    max_angular_z: float = 0.50
    publish_hz: float = 20.0
    key_timeout: float = 0.40
    layout: str = 'wasd'
    tty_device: str = '/dev/tty'

    def validate(self) -> None:
        """校验档位数值、非空终端与话题名称以及已支持的键盘布局.

        不验证话题是否接到预期节点，也不打开终端；非正有限超时和频率沿用
        禁用超时与最低频率逻辑，最小值和最大值之间的约束在整形时处理。

        Raises:
            ValueError: 参数类型错误、数值非有限、速度幅值为负、字符串为空或布局不支持。
        """
        for name, value in asdict(self).items():
            if name in ('cmd_vel_topic', 'tty_device'):
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f'{name} must be nonempty')
            elif name == 'layout':
                if value not in ('wasd', 'uiojkl'):
                    raise ValueError('layout must be wasd or uiojkl')
            elif (isinstance(value, bool) or not isinstance(value, (float, int))
                  or not math.isfinite(value)):
                raise ValueError(f'{name} must be finite')
            elif name not in ('publish_hz', 'key_timeout') and value < 0.0:
                raise ValueError(f'{name} must be nonnegative')

    @property
    def period_sec(self) -> float:
        """返回秒制循环周期，按旧逻辑把频率下限取为 1 Hz.

        Returns:
            1 除以 publish_hz 与 1.0 中的较大值；参数应先通过校验。
        """
        return 1.0 / max(self.publish_hz, 1.0)


def _lift(value: float, minimum: float, maximum: float) -> float:
    """先对称限幅，再提升非零小幅遥控命令，最小幅值不越过上限.

    零命令保持零，避免死区提升意外制造运动；输入有限性由调用方负责。

    Args:
        value: 线速度或角速度命令，与阈值使用相同单位。
        minimum: 最小非零幅值，取绝对值并限制到上限内。
        maximum: 对称幅值上限，使用其绝对值。

    Returns:
        保留方向、限幅且处理死区后的浮点命令。
    """
    limit = abs(float(maximum))
    limited = max(min(float(value), limit), -limit)
    if limited == 0.0:
        return 0.0
    minimum = min(abs(float(minimum)), limit)
    return math.copysign(minimum, limited) if abs(limited) < minimum else limited


def shape_motion(x: float, yaw: float, config: TeleopConfig) -> Tuple[float, float]:
    """按旧键盘逻辑将方向倍率转换为限幅并补偿死区的速度.

    Args:
        x: 前向方向倍率，通常为 -1、0 或 1。
        yaw: 偏航方向倍率，通常为 -1、0 或 1。
        config: 已校验的键盘档位、死区与上限参数。

    Returns:
        前向米每秒和偏航弧度每秒组成的二元组；不表示横向运动，
        零命令始终不被死区提升。
    """
    return (_lift(x * config.linear_speed, config.linear_deadzone, config.max_linear_x),
            _lift(yaw * config.angular_speed, config.angular_deadzone, config.max_angular_z))


def motion_for_key(key: str, layout: str = 'wasd') -> Optional[Tuple[float, float]]:
    """把完整按键事件转换为方向倍率，不调速也不切换全向运动模式.

    Args:
        key: 单个字符或完整转义序列；单字符按小写查表。
        layout: wasd 或 uiojkl，均接受方向键转义序列。

    Returns:
        前向与偏航倍率二元组；非运动事件返回 None。

    Raises:
        ValueError: 指定了不支持的布局。
    """
    if layout not in ('wasd', 'uiojkl'):
        raise ValueError('unsupported keyboard layout')
    if key in _ARROWS:
        return _ARROWS[key]
    lookup = key.lower() if len(key) == 1 else key
    return (_WASD if layout == 'wasd' else _UIOJKL).get(lookup)


class KeyboardState:
    """保存最新键盘意图和失活时刻，时钟及发布由运行层负责.

    Attributes:
        active: 当前前向、偏航速度意图，单位米每秒和弧度每秒。
        config: 已校验的键盘策略和速度限制。
        last_motion_time: 最后有效运动按键的单调时间，单位秒；零是未激活哨兵。
    """

    def __init__(self, config: TeleopConfig) -> None:
        """校验参数并初始化无活动命令的键盘状态.

        Args:
            config: 键盘参数，在运行层可能发布命令之前于此校验。

        Raises:
            ValueError: 键盘参数不合法。
        """
        config.validate()
        self.config = config
        self.active = (0.0, 0.0)
        self.last_motion_time = 0.0

    def handle_key(self, key: str, now: float) -> str:
        """更新按键意图，或返回由运行层执行停车与退出的事件.

        quit 只返回事件，不在此清除意图；stop 会清除意图和时间哨兵。
        未知键和空轮询均不刷新运动时间。

        Args:
            key: 已解析的完整字符或转义序列。
            now: 当前单调时间，单位秒；零值仍具有未激活哨兵语义。

        Returns:
            motion、stop、quit 或 ignored；运行层应在 stop 立即发零速，
            在 quit 走退出时的零速尝试，本纯状态对象不发布内容。
        """
        if key in ('q', 'Q', '\x03'):
            return 'quit'
        if key in (' ', 'k', 'K'):
            self.active = (0.0, 0.0)
            self.last_motion_time = 0.0
            return 'stop'
        motion = motion_for_key(key, self.config.layout)
        if motion is None:
            return 'ignored'
        self.active = shape_motion(*motion, self.config)
        self.last_motion_time = now
        return 'motion'

    def command(self, now: float) -> Tuple[float, float]:
        """按单调时间和严格超时规则返回活动意图或零速.

        超时只影响返回值，不清除 active 或 last_motion_time；没有额外的时钟回退处理。

        Args:
            now: 当前单调时间，单位秒。

        Returns:
            前向与偏航速度二元组；零时间哨兵或启用超时且年龄严格超过期限时为零，
            否则返回活动意图。
        """
        if self.last_motion_time == 0.0:
            return (0.0, 0.0)
        if self.config.key_timeout > 0.0 and now - self.last_motion_time > self.config.key_timeout:
            return (0.0, 0.0)
        return self.active
