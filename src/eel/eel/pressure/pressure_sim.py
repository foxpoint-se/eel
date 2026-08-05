from math import radians, tan
from time import time

from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import ImuStatus

from ..utils.topics import IMU_STATUS, MOTOR_CMD
from .pressure_source import PressureSource

TERMINAL_VELOCITY_MPS = 0.3
# Slight positive buoyancy: float up when not actively diving (rudder depth-control era).
FLOAT_VELOCITY_MPS = -0.05

MAX_DEPTH = 10.0
MIN_DEPTH = 0.0


def get_pitch_speed_velocity(terminal_velocity: float, pitch: float) -> float:
    return tan(radians(pitch)) * terminal_velocity


def calculate_position_delta(velocity_in_mps: float, time_in_s: float) -> float:
    return velocity_in_mps * time_in_s


def cap_depth(depth: float, min_depth: float, max_depth: float) -> float:
    if depth < min_depth:
        return min_depth
    if depth > max_depth:
        return max_depth
    return depth


class PressureSensorSimulator(PressureSource):
    def __init__(self, parent_node: Node) -> None:
        parent_node.create_subscription(ImuStatus, IMU_STATUS, self._handle_imu_msg, 10)
        parent_node.create_subscription(Float32, MOTOR_CMD, self.handle_motor_msg, 10)

        self._last_updated_at = time()
        self._current_motor_speed = 0.0
        self._current_depth = 0.0
        self._current_pitch = 0.0

        self.logger = parent_node.get_logger()

    def _handle_imu_msg(self, msg: ImuStatus) -> None:
        self._current_pitch = msg.pitch
        self._calculate_depth()

    def handle_motor_msg(self, msg: Float32) -> None:
        self._current_motor_speed = msg.data
        self._calculate_depth()

    def _calculate_depth(self) -> None:
        pitch_speed_velocity = (
            get_pitch_speed_velocity(TERMINAL_VELOCITY_MPS, self._current_pitch) * self._current_motor_speed
        )
        velocity = FLOAT_VELOCITY_MPS + pitch_speed_velocity

        now = time()
        if velocity != 0:
            time_delta = now - self._last_updated_at
            position_delta = calculate_position_delta(velocity, time_delta)
            new_position = self._current_depth + position_delta
            self._current_depth = cap_depth(new_position, MIN_DEPTH, MAX_DEPTH)

        self._last_updated_at = now

    def get_current_depth(self) -> float:
        self._calculate_depth()
        return self._current_depth
