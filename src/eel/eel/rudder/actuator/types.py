from typing import TypedDict


class Vector2d(TypedDict):
    x: float
    y: float


class ServoOptions(TypedDict):
    pin: int
    max_pulse_width: float
    min_pulse_width: float
    flip_direction: bool
    cap_min: float
    cap_max: float
    offset: float
