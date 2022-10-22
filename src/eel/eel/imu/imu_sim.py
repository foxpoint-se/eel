from rclpy.node import Node
from time import time
import random
from std_msgs.msg import Float32
from eel_interfaces.msg import TankStatus
from ..utils.sim import ANGULAR_VELOCITY
from ..utils.topics import (
    RUDDER_STATUS,
    MOTOR_CMD,
    FRONT_TANK_STATUS,
    REAR_TANK_STATUS,
)

TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS = 10.0
MOMENTUM_TOLERANCE = 0.03

# PITCH_NOISE_PERCENT = 0.001
# PITCH_RANGE_FLOOR = -90.0
# PITCH_RANGE_CEILING = 90.0
# PITCH_RANGE = abs(PITCH_RANGE_FLOOR) + abs(PITCH_RANGE_CEILING)
# PITCH_NOISE_ABS_VALUE = PITCH_NOISE_PERCENT * PITCH_RANGE
# PITCH_NOISE_FLOOR = -PITCH_NOISE_ABS_VALUE / 2
# PITCH_NOISE_CEILING = PITCH_NOISE_ABS_VALUE / 2
PITCH_NOISE_FLOOR = -0.1
PITCH_NOISE_CEILING = 0.1


def create_noise(floor, ceiling):
    return random.uniform(floor, ceiling)


def get_momentum_difference(front_tank_level, rear_tank_level):
    return front_tank_level - rear_tank_level


def get_velocity(terminal_velocity, fraction_of_velocity):
    return terminal_velocity * fraction_of_velocity


def cap_pitch(pitch):
    if pitch > 90.0:
        return 90.0
    elif pitch < -90.0:
        return -90.0
    return pitch


def calculate_angle_delta(angular_velocity, time_in_s):
    return angular_velocity * time_in_s


class ImuSimulator:
    def __init__(self, parent_node: Node) -> None:
        self.current_rudder_status = float(0)
        self.current_heading = float(0)
        self.speed = 0
        self._front_tank_level = 0.0
        self._rear_tank_level = 0.0
        self._current_pitch = 0.0
        self.last_updated_at = time()
        self.rudder_subscription = parent_node.create_subscription(
            Float32, RUDDER_STATUS, self._handle_rudder_msg, 10
        )
        self.motor_subscription = parent_node.create_subscription(
            Float32, MOTOR_CMD, self._handle_motor_msg, 10
        )

        parent_node.create_subscription(
            TankStatus, FRONT_TANK_STATUS, self._handle_front_tank_msg, 10
        )
        parent_node.create_subscription(
            TankStatus, REAR_TANK_STATUS, self._handle_rear_tank_msg, 10
        )

        self.imu_updater = parent_node.create_timer(
            1.0 / (parent_node.update_frequency * 2), self._loop
        )

    def _handle_rudder_msg(self, msg):
        self.current_rudder_status = msg.data

    def _handle_motor_msg(self, msg):
        self.speed = msg.data

    def _loop(self):
        self._update_heading()
        self._update_pitch()
        self.last_updated_at = time()

    def _update_pitch(self):
        momentum_difference = get_momentum_difference(
            self._front_tank_level, self._rear_tank_level
        )
        # if abs(momentum_difference) > MOMENTUM_TOLERANCE:
        now = time()
        time_delta = now - self.last_updated_at
        angular_velocity = get_velocity(
            TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS, momentum_difference
        )
        pitch_delta = calculate_angle_delta(angular_velocity, time_delta)
        new_pitch = (
            self._current_pitch
            + pitch_delta
            # + create_noise(PITCH_NOISE_FLOOR, PITCH_NOISE_CEILING)
        )
        capped_pitch = cap_pitch(new_pitch)
        self._current_pitch = capped_pitch

    def _update_heading(self):
        if self.speed > 0:
            now = time()
            time_delta = now - self.last_updated_at
            angle_delta = calculate_angle_delta(ANGULAR_VELOCITY, time_delta)
            to_add = self.current_rudder_status * angle_delta
            self.current_heading = (self.current_heading + to_add) % 360

    def _handle_front_tank_msg(self, msg):
        self._front_tank_level = msg.current_level

    def _handle_rear_tank_msg(self, msg):
        self._rear_tank_level = msg.current_level

    def get_calibration_status(self):
        sys = 3
        gyro = 3
        accel = 3
        mag = 3

        return sys, gyro, accel, mag

    def get_euler(self):
        return (
            self.current_heading,
            0.0,
            self._current_pitch + create_noise(PITCH_NOISE_FLOOR, PITCH_NOISE_CEILING),
        )

    def get_is_calibrated(self):
        return True
