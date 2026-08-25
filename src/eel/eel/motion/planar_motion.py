"""Shared motor+heading → planar Δmeters for sim plant and localization DR."""

from math import cos, radians, sin

DEFAULT_FORWARD_CRUISE_MPS = 1.0
DEFAULT_REVERSE_CRUISE_MPS = 0.2 * DEFAULT_FORWARD_CRUISE_MPS


def pitch_horizontal_scale(pitch_deg: float) -> float:
    """Fraction of body-axis speed that projects onto the horizontal plane."""
    return max(0.0, cos(radians(pitch_deg)))


def motor_to_speed_mps(
    motor: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
    pitch_deg: float = 0.0,
) -> float:
    if motor > 0.0:
        speed_mps = motor * forward_cruise_mps
    elif motor < 0.0:
        speed_mps = motor * reverse_cruise_mps
    else:
        return 0.0
    return speed_mps * pitch_horizontal_scale(pitch_deg)


def planar_delta_m(
    motor: float,
    heading_deg: float,
    dt_s: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
    pitch_deg: float = 0.0,
) -> tuple[float, float]:
    if dt_s <= 0.0:
        return 0.0, 0.0
    speed_mps = motor_to_speed_mps(
        motor,
        forward_cruise_mps=forward_cruise_mps,
        reverse_cruise_mps=reverse_cruise_mps,
        pitch_deg=pitch_deg,
    )
    if speed_mps == 0.0:
        return 0.0, 0.0
    yaw_rad = radians(heading_deg)
    return speed_mps * cos(yaw_rad) * dt_s, speed_mps * sin(yaw_rad) * dt_s
