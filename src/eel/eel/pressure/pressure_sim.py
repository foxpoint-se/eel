import random
from rclpy.node import Node
from eel_interfaces.msg import TankStatus
from time import time
from ..utils.topics import FRONT_TANK_STATUS, REAR_TANK_STATUS

NEUTRAL_LEVEL = 0.5
NEUTRAL_TOLERANCE = 0.02
TERMINAL_VELOCITY_MPS = 0.5

MAX_DEPTH = 10.0
MIN_DEPTH = 0.0

OS_ERROR_RATE = 0.01
# OS_ERROR_RATE = 0.0


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
    rear_offset = get_neutral_offset(
        rear_tank_level, neutral_level, neutral_tolerance
    )
    offset_average = (front_offset + rear_offset) / 2
    return offset_average


def get_velocity(terminal_velocity, fraction_of_velocity):
    return terminal_velocity * fraction_of_velocity


def calculate_position_delta(velocity_in_mps, time_in_s):
    return velocity_in_mps * time_in_s


def cap_depth(depth, min, max):
    if depth < min:
        return min
    if depth > max:
        return max
    return depth


class PressureSensorSimulator:
    def __init__(self, parent_node: Node) -> None:
        parent_node.create_subscription(
            TankStatus, FRONT_TANK_STATUS, self._handle_front_tank_msg, 10
        )
        parent_node.create_subscription(
            TankStatus, REAR_TANK_STATUS, self._handle_rear_tank_msg, 10
        )

        self._last_updated_at = time()
        self._current_depth = 0.0
        self._front_tank_level = 0.0
        self._rear_tank_level = 0.0

        self.logger = parent_node.get_logger()

    def _handle_front_tank_msg(self, msg):
        self._front_tank_level = msg.current_level
        self._calculate_depth()

    def _handle_rear_tank_msg(self, msg):
        self._rear_tank_level = msg.current_level
        self._calculate_depth()

    def _calculate_depth(self):
        average_bouyancy = get_average_bouyancy(
            self._front_tank_level,
            self._rear_tank_level,
            NEUTRAL_LEVEL,
            NEUTRAL_TOLERANCE,
        )
        velocity = get_velocity(TERMINAL_VELOCITY_MPS, average_bouyancy)
        if velocity != 0:
            now = time()
            time_delta = now - self._last_updated_at
            position_delta = calculate_position_delta(velocity, time_delta)
            new_position = self._current_depth + position_delta
            capped_depth = cap_depth(new_position, MIN_DEPTH, MAX_DEPTH)
            self._current_depth = capped_depth

        self._last_updated_at = time()

    def get_current_depth(self):
        if should_raise_oserror():
            raise OSError("Simulating OSError")

        return self._current_depth
