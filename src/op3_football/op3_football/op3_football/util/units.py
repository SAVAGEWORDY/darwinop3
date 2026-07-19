"""XM430 tick <-> angle conversions (same constants as L1 / ROBOTIS device file)."""

from __future__ import annotations

import math

VALUE_OF_0_RADIAN = 2048
VALUE_OF_MIN_RADIAN = 0
VALUE_OF_MAX_RADIAN = 4095
MIN_RADIAN = -math.pi
MAX_RADIAN = math.pi


def clamp_tick(value: int) -> int:
    return max(0, min(4095, int(value)))


def tick_to_radian(value: int) -> float:
    value = clamp_tick(value)
    if value > VALUE_OF_0_RADIAN:
        return (value - VALUE_OF_0_RADIAN) * MAX_RADIAN / (VALUE_OF_MAX_RADIAN - VALUE_OF_0_RADIAN)
    if value < VALUE_OF_0_RADIAN:
        return (value - VALUE_OF_0_RADIAN) * MIN_RADIAN / (VALUE_OF_MIN_RADIAN - VALUE_OF_0_RADIAN)
    return 0.0


def radian_to_tick(radian: float) -> int:
    if radian > 0.0:
        return int(radian * (VALUE_OF_MAX_RADIAN - VALUE_OF_0_RADIAN) / MAX_RADIAN + VALUE_OF_0_RADIAN)
    if radian < 0.0:
        return int(radian * (VALUE_OF_MIN_RADIAN - VALUE_OF_0_RADIAN) / MIN_RADIAN + VALUE_OF_0_RADIAN)
    return VALUE_OF_0_RADIAN


def tick_to_degree(value: int) -> float:
    return math.degrees(tick_to_radian(value))


def degree_to_tick(degree: float) -> int:
    return radian_to_tick(math.radians(degree))
