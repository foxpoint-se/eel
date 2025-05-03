import random
from math import tan, radians

from rclpy.node import Node
from eel_interfaces.msg import TankStatus, ImuStatus
from std_msgs.msg import Float32
from time import time
from .pressure_source import PressureSource
from ..utils.topics import FRONT_TANK_STATUS, REAR_TANK_STATUS, IMU_STATUS, MOTOR_CMD

NEUTRAL_LEVEL = 0.5
NEUTRAL_TOLERANCE = 0.02
TERMINAL_VELOCITY_MPS = 0.3

MAX_DEPTH = 10.0
MIN_DEPTH = 0.0

# TODO: remove this completely, if it turns out that we don't get OSErrors anymore
OS_ERROR_RATE = 0.0


def should_raise_oserror():
    return random.random() < OS_ERROR_RATE


def get_neutral_offset(tank_level, neutral_level, neutral_tolerance):
    neutral_ceiling = neutral_level + neutral_tolerance
    neutral_floor = neutral_level - neutral_tolerance
    if tank_level < neutral_floor:
        return tank_level - neutral_floor
    if tank_level > neutral_ceiling:
        return tank_level - neutral_ceiling
    return 0


def get_average_bouyancy(
    front_tank_level, rear_tank_level, neutral_level, neutral_tolerance
):
    front_offset = get_neutral_offset(
        front_tank_level, neutral_level, neutral_tolerance
    )
    rear_offset = get_neutral_offset(rear_tank_level, neutral_level, neutral_tolerance)
    offset_average = (front_offset + rear_offset) / 2
    return offset_average


def get_velocity(terminal_velocity, fraction_of_velocity):
    return terminal_velocity * fraction_of_velocity


def get_pitch_speed_velocity(terminal_velocity, pitch):
    return tan(radians(pitch)) * terminal_velocity


def calculate_position_delta(velocity_in_mps, time_in_s):
    return velocity_in_mps * time_in_s


def cap_depth(depth, min, max):
    if depth < min:
        return min
    if depth > max:
        return max
    return depth


class PressureSensorSimulator(PressureSource):
    def __init__(self, parent_node: Node) -> None:
        parent_node.create_subscription(
            TankStatus, FRONT_TANK_STATUS, self._handle_front_tank_msg, 10
        )
        parent_node.create_subscription(
            TankStatus, REAR_TANK_STATUS, self._handle_rear_tank_msg, 10
        )

        parent_node.create_subscription(
            ImuStatus, IMU_STATUS, self._handle_imu_msg, 10
        )

        parent_node.create_subscription(
            Float32, MOTOR_CMD, self.handle_motor_msg, 10
        )

        self._last_updated_at = time()
        self._current_motor_speed = 0.0
        self._current_depth = 0.0
        self._front_tank_level = 0.0
        self._rear_tank_level = 0.0
        self._current_pitch = 0.0

        self.logger = parent_node.get_logger()

    def _handle_front_tank_msg(self, msg):
        self._front_tank_level = msg.current_level
        self._calculate_depth()

    def _handle_rear_tank_msg(self, msg):
        self._rear_tank_level = msg.current_level
        self._calculate_depth()

    def _handle_imu_msg(self, msg):
        self._current_pitch = msg.pitch
        self._calculate_depth()

    def handle_motor_msg(self, msg):
        self._current_motor_speed = msg.data
        self._calculate_depth()

    def _calculate_depth(self):
        average_bouyancy = get_average_bouyancy(
            self._front_tank_level,
            self._rear_tank_level,
            NEUTRAL_LEVEL,
            NEUTRAL_TOLERANCE,
        )
        # NOTE: setting to negative here, so it will float up when motor not running
        tank_velocity =  -0.05 #get_velocity(TERMINAL_VELOCITY_MPS, average_bouyancy)
        pitch_speed_velocity = get_pitch_speed_velocity(TERMINAL_VELOCITY_MPS, self._current_pitch) * self._current_motor_speed
        velocity = tank_velocity + pitch_speed_velocity

        if velocity != 0:
            now = time()
            time_delta = now - self._last_updated_at
            position_delta = calculate_position_delta(velocity, time_delta)
            new_position = self._current_depth + position_delta
            capped_depth = cap_depth(new_position, MIN_DEPTH, MAX_DEPTH)
            self._current_depth = capped_depth

        self._last_updated_at = time()

    def get_current_depth(self):
        self._calculate_depth()

        if should_raise_oserror():
            raise OSError("Simulating OSError")

        return self._current_depth
