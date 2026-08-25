"""Horizontal motion for the world plant. No ROS — motor+heading in, x/y out."""

from eel.motion.planar_motion import (
    DEFAULT_THROTTLE_EXPONENT,
    motor_to_body_speed_mps,
    pitch_horizontal_scale,
    planar_delta_from_speed_mps,
    step_speed_toward_target,
)

# How fast body speed reaches motor target (1/s). Lower = longer coast when motor idles.
SIM_SPEED_RESPONSE_RATE_PER_S = 0.5


class MotionModel:
    """Integrates planar position from motor cmd and heading (deg)."""

    def __init__(self) -> None:
        self.x_m = 0.0
        self.y_m = 0.0
        self.motor_cmd = 0.0
        self.heading_deg = 0.0
        self.pitch_deg = 0.0
        self.throttle_exponent = DEFAULT_THROTTLE_EXPONENT
        self.speed_response_rate_per_s = SIM_SPEED_RESPONSE_RATE_PER_S
        self.current_body_speed_mps = 0.0

    def set_motor_cmd(self, motor_cmd: float) -> None:
        self.motor_cmd = motor_cmd

    def set_heading_deg(self, heading_deg: float) -> None:
        self.heading_deg = heading_deg

    def set_pitch_deg(self, pitch_deg: float) -> None:
        self.pitch_deg = pitch_deg

    def set_throttle_exponent(self, throttle_exponent: float) -> None:
        self.throttle_exponent = throttle_exponent

    def set_speed_response_rate_per_s(self, response_rate_per_s: float) -> None:
        self.speed_response_rate_per_s = max(0.0, response_rate_per_s)

    def _target_speed(self) -> float:
        return motor_to_body_speed_mps(
            self.motor_cmd,
            throttle_exponent=self.throttle_exponent,
        )

    def _apply_damping(self, target_body_speed_mps: float, dt_s: float) -> None:
        self.current_body_speed_mps = step_speed_toward_target(
            self.current_body_speed_mps,
            target_body_speed_mps,
            dt_s,
            response_rate_per_s=self.speed_response_rate_per_s,
        )

    def _integrate(self, dt_s: float) -> tuple[float, float]:
        horizontal_speed_mps = self.current_body_speed_mps * pitch_horizontal_scale(self.pitch_deg)
        dx, dy = planar_delta_from_speed_mps(horizontal_speed_mps, self.heading_deg, dt_s)
        self.x_m += dx
        self.y_m += dy
        return self.x_m, self.y_m

    def step(self, dt_s: float) -> tuple[float, float]:
        self._apply_damping(self._target_speed(), dt_s)
        return self._integrate(dt_s)
