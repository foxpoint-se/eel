"""Attitude physics for the world plant. No ROS — cmds in, heading/pitch out.

Lifted from eel.imu.imu_sim (same crude pitch/heading model).
"""

TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS = 12.5
MOMENTUM_TOLERANCE = 0.03
# Matches eel.motion.planar_motion DEFAULT_FORWARD_CRUISE_MPS (1.0 m/s).
ANGULAR_VELOCITY_DEGPS = 10.0
MAX_PITCH_DEG = 45.0


def momentum_difference(front_tank_level: float, rear_tank_level: float) -> float:
    return front_tank_level - 0.5 * rear_tank_level


def cap_pitch_deg(pitch_deg: float) -> float:
    return min(max(pitch_deg, -MAX_PITCH_DEG), MAX_PITCH_DEG)


class AttitudeModel:
    """Integrates heading and pitch from motor, rudder, and optional tank bias."""

    def __init__(self) -> None:
        self.heading_deg = 0.0
        self.pitch_deg = 0.0
        self.motor_cmd = 0.0
        self.rudder_x = 0.0
        self.rudder_y = 0.0
        self.front_tank_level = 0.0
        self.rear_tank_level = 0.0

    def set_motor_cmd(self, motor_cmd: float) -> None:
        self.motor_cmd = motor_cmd

    def set_rudder(self, rudder_x: float, rudder_y: float) -> None:
        self.rudder_x = rudder_x
        self.rudder_y = rudder_y

    def set_tank_levels(self, front_level: float, rear_level: float) -> None:
        self.front_tank_level = front_level
        self.rear_tank_level = rear_level

    def step(self, dt_s: float) -> tuple[float, float]:
        if dt_s <= 0.0:
            return self.heading_deg, self.pitch_deg

        self._step_heading(dt_s)
        self._step_pitch(dt_s)
        return self.heading_deg, self.pitch_deg

    def _step_heading(self, dt_s: float) -> None:
        if self.motor_cmd <= 0.0:
            return
        self.heading_deg = (self.heading_deg + self.rudder_x * ANGULAR_VELOCITY_DEGPS * dt_s) % 360.0

    def _step_pitch(self, dt_s: float) -> None:
        momentum = momentum_difference(self.front_tank_level, self.rear_tank_level)
        tank_delta = 0.0
        if abs(momentum) > MOMENTUM_TOLERANCE:
            tank_delta = TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS * momentum * dt_s

        rudder_delta = TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS * self.rudder_y * self.motor_cmd * dt_s
        self.pitch_deg = cap_pitch_deg(self.pitch_deg + tank_delta + rudder_delta)
