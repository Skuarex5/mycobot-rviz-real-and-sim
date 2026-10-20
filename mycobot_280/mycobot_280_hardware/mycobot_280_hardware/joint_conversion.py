"""Pure helpers to validate and convert myCobot joint angles (no ROS, no I/O)."""

import math
from typing import List, Optional, Sequence, Tuple

NUM_JOINTS = 6

DEFAULT_JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
# Defaults mirror the URDF limits (mycobot_description / Gazebo model).
DEFAULT_LOWER = [-2.9321, -2.4434, -2.6179, -2.6179, -2.7052, -3.14159]
DEFAULT_UPPER = [2.9321, 2.4434, 2.6179, 2.6179, 2.7925, 3.14159]


def validate_angles(raw, expected: int = NUM_JOINTS) -> Optional[List[float]]:
    """
    Return ``raw`` as a list of floats, or None if it is not a valid reading.

    A valid reading is a list/tuple with exactly ``expected`` finite real
    numbers (booleans are rejected). Anything else (None, error codes such as
    -1, empty lists, wrong length, NaN/inf, strings) yields None.
    """
    if not isinstance(raw, (list, tuple)) or len(raw) != expected:
        return None
    values = []
    for item in raw:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            return None
        if not math.isfinite(item):
            return None
        values.append(float(item))
    return values


def degrees_to_joint_positions(
    angles_deg: Sequence[float],
    offsets_rad: Sequence[float],
    signs: Sequence[float],
) -> List[float]:
    """Convert device degrees to URDF joint radians: ``sign * rad(deg) + offset``."""
    return [
        sign * math.radians(angle) + offset
        for angle, offset, sign in zip(angles_deg, offsets_rad, signs)
    ]


def joint_positions_to_degrees(
    positions_rad: Sequence[float],
    offsets_rad: Sequence[float],
    signs: Sequence[float],
) -> List[float]:
    """Convert URDF joint radians to device degrees: ``deg((pos - offset) / sign)``."""
    return [
        math.degrees((pos - offset) / sign)
        for pos, offset, sign in zip(positions_rad, offsets_rad, signs)
    ]


def check_signs(signs: Sequence[float]) -> bool:
    """Signs must be exactly +1 or -1."""
    return all(s in (1, -1) for s in signs)


def find_limit_violations(
    positions: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    tolerance: float = 0.0,
) -> List[Tuple[int, float]]:
    """Return ``(index, position)`` for joints outside ``[lower, upper]``."""
    return [
        (i, p)
        for i, (p, lo, hi) in enumerate(zip(positions, lower, upper))
        if p < lo - tolerance or p > hi + tolerance
    ]
