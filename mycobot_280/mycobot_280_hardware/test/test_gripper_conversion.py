import importlib.util
from pathlib import Path
import sys
import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


def conversion():
    assert importlib.util.find_spec("mycobot_280_hardware.gripper_conversion"), (
        "gripper conversion module is missing"
    )
    from mycobot_280_hardware import gripper_conversion

    return gripper_conversion


def test_calibration_preserves_nominal_mapping_and_safe_range():
    c = conversion()
    cal = c.GripperCalibration()
    assert c.radians_to_device_value(-0.5, cal) == 27
    assert c.radians_to_device_value(0.15, cal) == 100
    assert c.device_value_to_radians(0, cal) == pytest.approx(-0.74)
    for pos in (-0.5, -0.2, 0, 0.15):
        assert (
            abs(
                c.device_value_to_radians(c.radians_to_device_value(pos, cal), cal)
                - pos
            )
            <= 0.0045
        )
    for pos in (-0.74, 0.16, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            c.radians_to_device_value(pos, cal)
    for value in (-1, 101, None, float("nan"), True):
        with pytest.raises(ValueError):
            c.device_value_to_radians(value, cal)


def test_calibration_rejects_invalid_or_reversed_ranges():
    c = conversion()
    for kw in (
        {"closed_rad": 0.2},
        {"lower_rad": -0.8},
        {"open_value": 0},
        {"open_rad": float("nan")},
    ):
        with pytest.raises(ValueError):
            c.GripperCalibration(**kw)


def test_measured_calibration_endpoints_are_exact_without_rounding_past_limits():
    c = conversion()
    calibration = c.GripperCalibration()
    assert c.device_value_to_radians(100, calibration) == calibration.open_rad
    assert c.device_value_to_radians(0, calibration) == calibration.closed_rad
