import math

import pytest

from mycobot_280_hardware.joint_conversion import (
    check_signs,
    degrees_to_joint_positions,
    find_limit_violations,
    joint_positions_to_degrees,
    validate_angles,
)


@pytest.mark.parametrize('raw', [
    None, -1, [], [0] * 5, [0] * 7, 'abcdef', [0, 0, 0, 0, 0, None],
    [0, 0, 0, 0, 0, 'x'], [0, 0, 0, 0, 0, float('nan')],
    [0, 0, 0, 0, 0, float('inf')], [True, 0, 0, 0, 0, 0],
])
def test_validate_rejects_invalid(raw):
    assert validate_angles(raw) is None


def test_validate_accepts_valid():
    assert validate_angles([0, 1, -2.5, 90, 180, -180.0]) == [
        0.0, 1.0, -2.5, 90.0, 180.0, -180.0]
    assert validate_angles((0,) * 6) == [0.0] * 6


def test_conversion_identity_calibration():
    out = degrees_to_joint_positions([180, 90, 0, -90, 45, 0], [0.0] * 6, [1] * 6)
    assert out == pytest.approx([math.pi, math.pi / 2, 0, -math.pi / 2, math.pi / 4, 0])


def test_conversion_offsets_and_signs():
    offsets = [0.1, -0.2, 0.0, 0.05, -0.1, 0.2]
    signs = [-1, 1, -1, 1, 1, -1]
    deg_in = [90.0, -45.0, 30.0, 0.0, 15.0, -60.0]
    rad = degrees_to_joint_positions(deg_in, offsets, signs)
    assert rad[0] == pytest.approx(-math.pi / 2 + 0.1)
    assert rad[1] == pytest.approx(-math.pi / 4 - 0.2)
    deg_roundtrip = joint_positions_to_degrees(rad, offsets, signs)
    assert deg_roundtrip == pytest.approx(deg_in, abs=1e-9)


def test_signs():
    assert check_signs([1, -1, 1.0, -1.0, 1, 1])
    assert not check_signs([1, 0, 1, 1, 1, 1])
    assert not check_signs([1, 2, 1, 1, 1, 1])


def test_limit_violations():
    lo, hi = [-1.0] * 6, [1.0] * 6
    assert find_limit_violations([0.0] * 6, lo, hi) == []
    assert find_limit_violations([0, 1.5, 0, 0, -1.5, 0], lo, hi) == [
        (1, 1.5), (4, -1.5)]
