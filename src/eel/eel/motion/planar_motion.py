"""Shared motor+heading → planar Δmeters for sim plant and localization DR."""

from math import cos, radians, sin

DEFAULT_FORWARD_CRUISE_MPS = 1.0
DEFAULT_REVERSE_CRUISE_MPS = 0.2 * DEFAULT_FORWARD_CRUISE_MPS


def motor_to_speed_mps(
    motor: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
) -> float:
    if motor > 0.0:
        return motor * forward_cruise_mps
    if motor < 0.0:
        return motor * reverse_cruise_mps
    return 0.0


def planar_delta_m(
    motor: float,
    heading_deg: float,
    dt_s: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
) -> tuple[float, float]:
    if dt_s <= 0.0:
        return 0.0, 0.0
    speed_mps = motor_to_speed_mps(
        motor,
        forward_cruise_mps=forward_cruise_mps,
        reverse_cruise_mps=reverse_cruise_mps,
    )
    if speed_mps == 0.0:
        return 0.0, 0.0
    yaw_rad = radians(heading_deg)
    return speed_mps * cos(yaw_rad) * dt_s, speed_mps * sin(yaw_rad) * dt_s
