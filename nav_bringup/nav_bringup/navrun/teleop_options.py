"""Explicit keyboard-only overrides shared by preview, rendering and replay."""

from dataclasses import asdict, dataclass
import math
from typing import Any, Dict, Mapping, Optional

from nav_bringup.navcfg import render


@dataclass(frozen=True)
class TeleopOptions:
    """Optional per-run selectors; omitted fields retain the selected profile.

    Attributes:
        layout: wasd or uiojkl, without adding keys or changing topic ownership.
        linear_speed: Nonnegative requested gear in m/s, within profile limits.
        angular_speed: Nonnegative requested gear in rad/s, within profile limits.
    """

    layout: Optional[str] = None
    linear_speed: Optional[float] = None
    angular_speed: Optional[float] = None

    def values(self, mode: str) -> Dict[str, Any]:
        """Validate selectors before resource discovery or host operations.

        Args:
            mode: Selected startup mode; nonempty overrides require teleop.

        Returns:
            Only explicitly provided options, suitable for freezing as YAML.

        Raises:
            ValueError: A selector is invalid or used outside teleop mode.
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
    """Apply explicit gears without changing source data or safety ceilings.

    Profile-owned deadzone lifting and clamping remain unchanged at runtime.
    Unlike existing profile gears, explicitly requested excessive gears fail
    early, instead of being silently clamped after an operator requests them.

    Args:
        profile: Robot profile after the selected mode overlay.
        mode: Startup mode, used to reject misplaced overrides.
        options: Immutable explicit selectors, never arbitrary profile edits.

    Returns:
        A new profile with validated keyboard-only overrides applied.

    Raises:
        ValueError: An override is malformed or exceeds a profile-owned ceiling.
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
