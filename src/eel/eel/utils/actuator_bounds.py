import math

from .utils import clamp

MOTOR_CMD_MIN = -1.0
MOTOR_CMD_MAX = 1.0
TANK_LEVEL_MIN = 0.0
TANK_LEVEL_MAX = 1.0
MAX_DEPTH_M = 3.0
MAX_PITCH_DEG = 45.0
LAT_MIN = -90.0
LAT_MAX = 90.0
LON_MIN = -180.0
LON_MAX = 180.0


def is_finite_number(value: float) -> bool:
    return math.isfinite(value)


def bounded_unit_cmd(value: float) -> float | None:
    if not is_finite_number(value):
        return None
    return clamp(value, MOTOR_CMD_MIN, MOTOR_CMD_MAX)


def bounded_tank_level(value: float) -> float | None:
    if not is_finite_number(value):
        return None
    return clamp(value, TANK_LEVEL_MIN, TANK_LEVEL_MAX)


def is_valid_depth_target(value: float, max_depth_m: float = MAX_DEPTH_M) -> bool:
    if not is_finite_number(value):
        return False
    return 0.0 <= value <= max_depth_m


def bounded_depth_target(value: float, max_depth_m: float = MAX_DEPTH_M) -> float | None:
    if not is_finite_number(value):
        return None
    return clamp(value, 0.0, max_depth_m)


def bounded_pitch_target(value: float, max_pitch_deg: float = MAX_PITCH_DEG) -> float | None:
    if not is_finite_number(value):
        return None
    return clamp(value, -max_pitch_deg, max_pitch_deg)


def parse_depth_control_targets(depth_target: float, pitch_target: float) -> tuple[float, float] | None:
    depth = bounded_depth_target(depth_target)
    pitch = bounded_pitch_target(pitch_target)
    if depth is None or pitch is None:
        return None
    return depth, pitch


def is_valid_coordinate(lat: float, lon: float) -> bool:
    if not is_finite_number(lat) or not is_finite_number(lon):
        return False
    return LAT_MIN <= lat <= LAT_MAX and LON_MIN <= lon <= LON_MAX
