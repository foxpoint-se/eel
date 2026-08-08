"""Depth physics for the world plant. No ROS — numbers in, depth out.

Lifted from eel.pressure.pressure_sim (same crude dive model).
"""

from math import radians, tan

TERMINAL_VELOCITY_MPS = 0.3
FLOAT_VELOCITY_MPS = -0.05  # slight positive buoyancy
MAX_DEPTH_M = 10.0
MIN_DEPTH_M = 0.0


def _pitch_speed_velocity_mps(terminal_velocity_mps: float, pitch_deg: float) -> float:
    return tan(radians(pitch_deg)) * terminal_velocity_mps


def _cap_depth_m(depth_m: float) -> float:
    return min(max(depth_m, MIN_DEPTH_M), MAX_DEPTH_M)


class DepthModel:
    """Integrates depth from pitch (deg) and motor cmd over time."""

    def __init__(self) -> None:
        self.depth_m = 0.0
        self.pitch_deg = 0.0
        self.motor_cmd = 0.0

    def set_pitch_deg(self, pitch_deg: float) -> None:
        self.pitch_deg = pitch_deg

    def set_motor_cmd(self, motor_cmd: float) -> None:
        self.motor_cmd = motor_cmd

    def step(self, dt_s: float) -> float:
        if dt_s <= 0.0:
            return self.depth_m

        dive_velocity_mps = (
            _pitch_speed_velocity_mps(TERMINAL_VELOCITY_MPS, self.pitch_deg) * self.motor_cmd
        )
        velocity_mps = FLOAT_VELOCITY_MPS + dive_velocity_mps
        if velocity_mps != 0.0:
            self.depth_m = _cap_depth_m(self.depth_m + velocity_mps * dt_s)
        return self.depth_m
