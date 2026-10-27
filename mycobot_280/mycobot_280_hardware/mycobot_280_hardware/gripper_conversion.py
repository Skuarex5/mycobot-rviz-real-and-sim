"""Nominal device-to-model calibration; not a measurement of jaw width."""

from dataclasses import dataclass
import math
from numbers import Real

GRIPPER_JOINT = "gripper_controller"
GRIPPER_ACTION = "mycobot280m5_gripper_controller/follow_joint_trajectory"


@dataclass(frozen=True)
class GripperCalibration:
    closed_rad: float = -0.74
    open_rad: float = 0.15
    closed_value: float = 0.0
    open_value: float = 100.0
    lower_rad: float = -0.5
    upper_rad: float = 0.15

    def __post_init__(self):
        if not all(
            isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v)
            for v in self.__dict__.values()
        ):
            raise ValueError("Calibration values must be finite numbers")
        if not (
            self.closed_rad < self.open_rad
            and 0 <= self.closed_value < self.open_value <= 100
            and self.closed_rad <= self.lower_rad < self.upper_rad <= self.open_rad
        ):
            raise ValueError("Invalid gripper calibration ranges")
        if self.lower_rad < -0.5 or self.upper_rad > 0.15:
            raise ValueError("Calibration cannot expand the model mimic limits")


def device_value_to_radians(value: float, calibration: GripperCalibration) -> float:
    # Preserve measured values even outside model range: never fabricate/clamp state.
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or not 0 <= value <= 100
    ):
        raise ValueError("Invalid measured gripper value (expected 0..100)")
    c = calibration
    # Exact calibration endpoints avoid arithmetic roundoff outside model limits.
    # Values between/outside those endpoints retain their measured interpolation.
    if value == c.open_value:
        return c.open_rad
    if value == c.closed_value:
        return c.closed_rad
    return c.closed_rad + (value - c.closed_value) * (c.open_rad - c.closed_rad) / (
        c.open_value - c.closed_value
    )


def radians_to_device_value(position: float, calibration: GripperCalibration) -> int:
    c = calibration
    if (
        isinstance(position, bool)
        or not isinstance(position, Real)
        or not math.isfinite(position)
        or not c.lower_rad <= position <= c.upper_rad
    ):
        raise ValueError("Gripper target is outside compatible mimic limits")
    value = c.closed_value + (position - c.closed_rad) * (
        c.open_value - c.closed_value
    ) / (c.open_rad - c.closed_rad)
    result = round(value)
    if not 0 <= result <= 100:
        raise ValueError("Gripper target maps outside device range")
    return result
