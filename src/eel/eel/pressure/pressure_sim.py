import random
from math import tan, radians

from rclpy.node import Node
from eel_interfaces.msg import TankStatus, ImuStatus
from std_msgs.msg import Float32
from time import time, monotonic
from .pressure_source import PressureSource
from ..utils.topics import FRONT_TANK_STATUS, REAR_TANK_STATUS, IMU_STATUS, MOTOR_CMD

# near top
RHO = 1000.0
G = 9.81
TANK_VOLUME_MAX = 0.00035  # m^3 -- set to your value
VELOCITY_SCALE = 0.25  # tune later
IYY = 5.0  # not used exactly here, optional

NEUTRAL_LEVEL = 0.5
NEUTRAL_TOLERANCE = 0.001
TERMINAL_VELOCITY_MPS = 0.3

MAX_DEPTH = 10.0
MIN_DEPTH = 0.0

# TODO: remove this completely, if it turns out that we don't get OSErrors anymore
OS_ERROR_RATE = 0.0


def should_raise_oserror():
    return random.random() < OS_ERROR_RATE


Lf = 0.40  # Match your controller
Lr = 0.10  # Match your controller


def get_average_bouyancy(
    front_tank_level, rear_tank_level, neutral_level, neutral_tolerance
) -> float:
    # compute fill deviations from neutral
    front_dev = front_tank_level - neutral_level
    rear_dev = rear_tank_level - neutral_level

    # optionally apply small deadband (but make it tiny)
    if abs(front_dev) < 0.001:
        front_dev = 0.0
    if abs(rear_dev) < 0.001:
        rear_dev = 0.0

    # net added volume (m^3) relative to one tank max is:
    # net_vol = (front_dev + rear_dev) * TANK_VOLUME_MAX
    # but here we return normalized fraction so get_velocity can use it directly:
    net_fraction = (front_dev + rear_dev) / 2.0  # average deviation fraction [-1..1]
    return net_fraction


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

        parent_node.create_subscription(ImuStatus, IMU_STATUS, self._handle_imu_msg, 10)

        parent_node.create_subscription(Float32, MOTOR_CMD, self.handle_motor_msg, 10)

        self._last_updated_at = time()
        self._current_motor_speed = 0.0
        self._current_depth = 0.0
        self._front_tank_level = NEUTRAL_LEVEL
        self._rear_tank_level = NEUTRAL_LEVEL
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

    def _calculate_depth(self) -> None:
        """Update simulated depth based on buoyancy and pitch."""
        now = monotonic()
        dt = now - getattr(self, "_last_update_time", now)
        self._last_update_time = now

        # buoyancy deviation in [-1..1]
        avg_buoy = get_average_bouyancy(
            self._front_tank_level,
            self._rear_tank_level,
            NEUTRAL_LEVEL,
            NEUTRAL_TOLERANCE,
        )

        # map buoyancy fraction to vertical velocity (m/s)
        # TERMINAL_VELOCITY_MPS = e.g. 0.2
        velocity_from_buoy = TERMINAL_VELOCITY_MPS * avg_buoy

        # pitch adds a vertical component from "forward" motion
        velocity_from_pitch = (
            tan(radians(self._current_pitch))
            * TERMINAL_VELOCITY_MPS
            * self._current_motor_speed
        )

        # total vertical velocity
        velocity = velocity_from_buoy + velocity_from_pitch

        # integrate depth (down positive)
        self._current_depth += velocity * dt

    def get_current_depth(self):
        self._calculate_depth()

        if should_raise_oserror():
            raise OSError("Simulating OSError")

        return self._current_depth
