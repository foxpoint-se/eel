"""Pure rudder helpers (roll compensation, offset → servo command)."""

import math

from ..utils.utils import clamp
from .actuator.types import Vector2d


def rotate_vector(vector: Vector2d, rotation_degrees: float) -> Vector2d:
    rotation_radians = math.radians(rotation_degrees)
    x = vector["x"]
    y = vector["y"]
    rotated_x = math.cos(rotation_radians) * x - math.sin(rotation_radians) * y
    rotated_y = math.sin(rotation_radians) * x + math.cos(rotation_radians) * y
    return {"x": rotated_x, "y": rotated_y}


def roll_compensated_deflection(direction: Vector2d, roll_degrees: float) -> Vector2d:
    compensated = rotate_vector(direction, -roll_degrees)
    return {
        "x": float(clamp(compensated["x"], -1, 1)),
        "y": float(clamp(compensated["y"], -1, 1)),
    }


def servo_command_with_offset(
    deflection: float,
    offset: float,
    cap_min: float,
    cap_max: float,
) -> float:
    return float(clamp(deflection + offset, cap_min, cap_max))
