"""只针对键盘遥控的显式覆盖项，预览、渲染与回放共用同一份定义.

覆盖只改这一次运行的键位与档位，不改源档案，也不放宽档案自带的安全上限。
"""

from dataclasses import asdict, dataclass
import math
from typing import Any, Dict, Mapping, Optional

from nav_bringup.navcfg import render


@dataclass(frozen=True)
class TeleopOptions:
    """可选的单次运行选择子；没写的字段一律沿用选定档案.

    Attributes:
        layout: wasd 或 uiojkl；只换键位，不新增按键，也不改变话题归属。
        linear_speed: 请求的线速度档位，非负、单位 m/s，必须在档案上限内。
        angular_speed: 请求的角速度档位，非负、单位 rad/s，必须在档案上限内。
    """

    layout: Optional[str] = None
    linear_speed: Optional[float] = None
    angular_speed: Optional[float] = None

    def values(self, mode: str) -> Dict[str, Any]:
        """在资源发现与主机操作之前先校验选择子.

        Args:
            mode: 选定的启动模式；非空覆盖只允许出现在遥控模式。

        Returns:
            只包含显式给出的选项，可直接冻结成 YAML；没给的字段不会被补默认值。

        Raises:
            ValueError: 选择子非法，或在非遥控模式下使用。
        """
        values = {key: value for key, value in asdict(self).items() if value is not None}
        if values and mode != 'teleop':
            raise ValueError('keyboard overrides require teleop mode')
        if self.layout is not None and self.layout not in ('wasd', 'uiojkl'):
            raise ValueError('teleop layout must be wasd or uiojkl')
        for key in ('linear_speed', 'angular_speed'):
            value = values.get(key)
            if value is not None and (type(value) not in (int, float)
                                      or not math.isfinite(value) or value < 0):
                raise ValueError(f'teleop {key} must be finite and nonnegative')
        return values


def apply_options(
    profile: Mapping[str, Any], mode: str, options: TeleopOptions,
) -> Dict[str, Any]:
    """套用显式档位，不改动源数据，也不放宽安全上限.

    档案自带的死区抬升与限幅在运行期保持原样。与档案里既有的档位不同，
    显式请求超限的档位会当场失败，而不是在操作者提出之后被静默限幅——
    这样操作者不会以为自己拿到了更高的速度。

    Args:
        profile: 叠加选定模式差异之后的机器人档案。
        mode: 启动模式，用于拒绝放错位置的覆盖。
        options: 不可变的显式选择子，不是任意档案编辑入口。

    Returns:
        套用了已校验的键盘覆盖之后的新档案（原档案对象不被修改）。

    Raises:
        ValueError: 覆盖项格式错误，或超过档案自带的上限。
    """
    values = options.values(mode)
    for key, limit in (('linear_speed', 'linear_x'), ('angular_speed', 'angular_z')):
        if key not in values:
            continue
        for path in ('teleop.max_' + limit, 'chassis.adapter.hard_max_' + limit):
            ceiling = render.profile_value(profile, path)
            if (type(ceiling) not in (int, float) or not math.isfinite(ceiling)
                    or ceiling < 0 or values[key] > ceiling):
                raise ValueError(f'teleop {key} exceeds or has invalid limit {path}')
    return render.deep_merge(profile, {'teleop': values} if values else {})
