from rclpy.node import Node
from time import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from eel_interfaces.msg import TankStatus
from .types import CalibrationOffsets
from ..utils.sim import ANGULAR_VELOCITY
from ..utils.topics import (
    RUDDER_STATUS,
    MOTOR_CMD,
    FRONT_TANK_STATUS,
    REAR_TANK_STATUS,
)


TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS = 12.5
MOMENTUM_TOLERANCE = 0.005


# Imagine a sea-saw with forces F1, F2 on each end, L and 2L from the centre.
#
# Rear                      Front
#
#      F1 < L >  < --- 2L -->  F2
# ------------------------------
#               ^
#
# For balance: F1 * L = F2 * 2L
#
# Simplify, divide by L: F1 = 2 F2
#
# Balanced when: 2 F2 - F1 = 0
#
# Difference in momentum: 2 F2 - F1 = ?
#
# Or: F2 - 0.5 F1 = ?
#
def get_momentum_difference(front_tank_level, rear_tank_level):
    # return front_tank_level - 0.5 * rear_tank_level
    return front_tank_level - rear_tank_level


def get_velocity(terminal_velocity, fraction_of_velocity):
    return terminal_velocity * fraction_of_velocity


def cap_pitch(pitch):
    if pitch > 45.0:
        return 45.0
    elif pitch < -45.0:
        return -45.0
    return pitch


def calculate_angle_delta(angular_velocity, time_in_s):
    return angular_velocity * time_in_s


class ImuSimulator:
    def __init__(self, parent_node: Node) -> None:
        self.current_rudder_status = Vector3()
        self.current_heading = float(0)
        self.speed = 0
        self._front_tank_level = 0.0
        self._rear_tank_level = 0.0
        self._current_pitch = 0.0
        self.last_updated_at = time()
        self.sensor_offsets = {"mag": (0, 0, 0), "gyr": (0, 0, 0), "acc": (0, 0, 0)}

        self.rudder_subscription = parent_node.create_subscription(
            Vector3, RUDDER_STATUS, self._handle_rudder_msg, 10
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

    def _handle_rudder_msg(self, msg: Vector3):
        self.current_rudder_status = msg

    def _handle_motor_msg(self, msg):
        self.speed = msg.data

    def _loop(self):
        self._update_heading()
        self._update_pitch()
        self.last_updated_at = time()

    def _update_pitch(self):
        now = time()
        time_delta = now - self.last_updated_at
        momentum_difference = get_momentum_difference(
            self._front_tank_level, self._rear_tank_level
        )

        tank_pitch_delta = 0.0
        if abs(momentum_difference) > MOMENTUM_TOLERANCE:
            angular_velocity = get_velocity(
                TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS, momentum_difference
            )
            tank_pitch_delta = calculate_angle_delta(angular_velocity, time_delta)

        rudder_velocity = (
            get_velocity(
                TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS, self.current_rudder_status.y
            )
            * self.speed
        )
        rudder_pitch_delta = calculate_angle_delta(rudder_velocity, time_delta)

        new_pitch = self._current_pitch + tank_pitch_delta + rudder_pitch_delta
        capped_pitch = cap_pitch(new_pitch)
        self._current_pitch = capped_pitch

    def _update_heading(self):
        if self.speed > 0:
            now = time()
            time_delta = now - self.last_updated_at
            angle_delta = calculate_angle_delta(ANGULAR_VELOCITY, time_delta)
            to_add = self.current_rudder_status.x * angle_delta
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
        return self.current_heading, 0.0, self._current_pitch

    def get_is_calibrated(self):
        return True

    def get_calibration_offsets(self) -> CalibrationOffsets:
        return self.sensor_offsets

    def set_offset_values(self, offset_mapping: CalibrationOffsets):
        self.sensor_offsets.update(offset_mapping)
