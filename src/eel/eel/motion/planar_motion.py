"""Shared motor+heading → planar Δmeters for sim plant and localization DR.

Heading is compass bearing (0° = north, 90° = east). Returns (east_m, north_m).
"""

from math import copysign, cos, radians, sin

DEFAULT_FORWARD_CRUISE_MPS = 1.0
DEFAULT_REVERSE_CRUISE_MPS = 0.2 * DEFAULT_FORWARD_CRUISE_MPS

# Throttle curve exponent: 1 = linear; >1 reduces speed at partial throttle.
DEFAULT_THROTTLE_EXPONENT = 1.0


def pitch_horizontal_scale(pitch_deg: float) -> float:
    """Fraction of body-axis speed that projects onto the horizontal plane."""
    return max(0.0, cos(radians(pitch_deg)))


def nonlinear_motor_fraction(motor: float, *, throttle_exponent: float = DEFAULT_THROTTLE_EXPONENT) -> float:
    if motor == 0.0:
        return 0.0
    if throttle_exponent == 1.0:
        return motor
    return float(copysign(abs(motor) ** throttle_exponent, motor))


def motor_to_body_speed_mps(
    motor: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
    throttle_exponent: float = DEFAULT_THROTTLE_EXPONENT,
) -> float:
    scaled_motor = nonlinear_motor_fraction(motor, throttle_exponent=throttle_exponent)
    if scaled_motor > 0.0:
        return scaled_motor * forward_cruise_mps
    if scaled_motor < 0.0:
        return scaled_motor * reverse_cruise_mps
    return 0.0


def motor_to_speed_mps(
    motor: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
    pitch_deg: float = 0.0,
    throttle_exponent: float = DEFAULT_THROTTLE_EXPONENT,
) -> float:
    return motor_to_body_speed_mps(
        motor,
        forward_cruise_mps=forward_cruise_mps,
        reverse_cruise_mps=reverse_cruise_mps,
        throttle_exponent=throttle_exponent,
    ) * pitch_horizontal_scale(pitch_deg)


def step_speed_toward_target(
    current_speed_mps: float,
    target_speed_mps: float,
    dt_s: float,
    *,
    response_rate_per_s: float,
) -> float:
    """Move current speed toward target. rate 0 = instant; lower rate = longer coast."""
    if dt_s <= 0.0:
        return current_speed_mps
    if response_rate_per_s <= 0.0:
        return target_speed_mps
    blend = min(1.0, response_rate_per_s * dt_s)
    return current_speed_mps + (target_speed_mps - current_speed_mps) * blend


def planar_delta_from_speed_mps(
    speed_mps: float,
    heading_deg: float,
    dt_s: float,
) -> tuple[float, float]:
    if dt_s <= 0.0 or speed_mps == 0.0:
        return 0.0, 0.0
    bearing_rad = radians(heading_deg)
    east_m = speed_mps * sin(bearing_rad) * dt_s
    north_m = speed_mps * cos(bearing_rad) * dt_s
    return east_m, north_m


def enu_yaw_deg_from_compass_bearing(heading_deg: float) -> float:
    """ROS ENU yaw for odom/TF: 0° = east, 90° = north. Heading is compass bearing."""
    return (90.0 - heading_deg) % 360.0


def planar_delta_m(
    motor: float,
    heading_deg: float,
    dt_s: float,
    *,
    forward_cruise_mps: float = DEFAULT_FORWARD_CRUISE_MPS,
    reverse_cruise_mps: float = DEFAULT_REVERSE_CRUISE_MPS,
    pitch_deg: float = 0.0,
    throttle_exponent: float = DEFAULT_THROTTLE_EXPONENT,
) -> tuple[float, float]:
    if dt_s <= 0.0:
        return 0.0, 0.0
    speed_mps = motor_to_speed_mps(
        motor,
        forward_cruise_mps=forward_cruise_mps,
        reverse_cruise_mps=reverse_cruise_mps,
        pitch_deg=pitch_deg,
        throttle_exponent=throttle_exponent,
    )
    if speed_mps == 0.0:
        return 0.0, 0.0
    return planar_delta_from_speed_mps(speed_mps, heading_deg, dt_s)
