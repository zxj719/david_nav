"""Legacy-equivalent keyboard command state, with no ROS or terminal operations."""

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
    """Startup parameters, retaining the old standalone keyboard defaults.

    Attributes:
        cmd_vel_topic: Output topic feeding the adapter, never the driver directly.
        linear_speed: Selected forward/reverse gear magnitude.
        angular_speed: Selected yaw gear magnitude.
        linear_deadzone: Minimum nonzero linear command.
        angular_deadzone: Minimum nonzero yaw command.
        max_linear_x: Keyboard linear magnitude ceiling.
        max_angular_z: Keyboard angular magnitude ceiling.
        publish_hz: Loop frequency, floored at one hertz.
        key_timeout: Monotonic-time inactivity timeout; nonpositive disables it.
        layout: wasd or uiojkl, both also supporting arrows.
        tty_device: Explicit terminal device, independent of redirected stdin.
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
        """Validate finite gears and supported terminal/layout selections.

        Raises:
            ValueError: A parameter is malformed or a magnitude is negative.
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
        """Return the legacy loop period with its one-hertz floor."""
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
    """Apply the same signed deadzone and ceiling logic as the legacy keyboard.

    Args:
        x: Forward direction multiplier.
        yaw: Yaw direction multiplier.
        config: Validated keyboard gear settings.

    Returns:
        Forward and yaw command values; no lateral motion is represented.
    """
    return (_lift(x * config.linear_speed, config.linear_deadzone, config.max_linear_x),
            _lift(yaw * config.angular_speed, config.angular_deadzone, config.max_angular_z))


def motion_for_key(key: str, layout: str = 'wasd') -> Optional[Tuple[float, float]]:
    """Translate a complete key event without speed adjustment or omnidirectional modes.

    Args:
        key: One character or a complete escape sequence.
        layout: wasd or uiojkl.

    Returns:
        Forward/yaw multipliers, or None for a non-motion event.

    Raises:
        ValueError: An unsupported layout is requested.
    """
    if layout not in ('wasd', 'uiojkl'):
        raise ValueError('unsupported keyboard layout')
    if key in _ARROWS:
        return _ARROWS[key]
    lookup = key.lower() if len(key) == 1 else key
    return (_WASD if layout == 'wasd' else _UIOJKL).get(lookup)


class KeyboardState:
    """A bounded latest-command state; the runtime owns clocks and publications.

    Attributes:
        active: 当前前向/偏航速度意图（米/秒、弧度/秒）。
        config: 已校验的键盘策略和速度限制。
        last_motion_time: 最后有效运动按键的单调时间（秒）；零值表示未激活。
    """

    def __init__(self, config: TeleopConfig) -> None:
        """Start with no active command.

        Args:
            config: Keyboard parameters checked before any eventual publication.
        """
        config.validate()
        self.config = config
        self.active = (0.0, 0.0)
        self.last_motion_time = 0.0

    def handle_key(self, key: str, now: float) -> str:
        """Update motion intent or return a stop/quit event for the caller to execute.

        Args:
            key: Complete key event; an empty poll never refreshes freshness.
            now: Current monotonic time in seconds.

        Returns:
            motion, stop, quit or ignored. Stop requires immediate zero publication
            by the eventual runtime, and quit requires its shutdown stop burst.
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
        """Return the active command or zero after monotonic keyboard inactivity.

        Args:
            now: Current monotonic time in seconds.

        Returns:
            Forward/yaw pair, preserving the legacy strict age-greater-than check.
        """
        if self.last_motion_time == 0.0:
            return (0.0, 0.0)
        if self.config.key_timeout > 0.0 and now - self.last_motion_time > self.config.key_timeout:
            return (0.0, 0.0)
        return self.active
