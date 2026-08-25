"""Horizontal motion for the world plant. No ROS — motor+heading in, x/y out."""

from eel.motion.planar_motion import planar_delta_m


class MotionModel:
    """Integrates planar position from motor cmd and heading (deg)."""

    def __init__(self) -> None:
        self.x_m = 0.0
        self.y_m = 0.0
        self.motor_cmd = 0.0
        self.heading_deg = 0.0
        self.pitch_deg = 0.0

    def set_motor_cmd(self, motor_cmd: float) -> None:
        self.motor_cmd = motor_cmd

    def set_heading_deg(self, heading_deg: float) -> None:
        self.heading_deg = heading_deg

    def set_pitch_deg(self, pitch_deg: float) -> None:
        self.pitch_deg = pitch_deg

    def step(self, dt_s: float) -> tuple[float, float]:
        dx, dy = planar_delta_m(self.motor_cmd, self.heading_deg, dt_s, pitch_deg=self.pitch_deg)
        self.x_m += dx
        self.y_m += dy
        return self.x_m, self.y_m
