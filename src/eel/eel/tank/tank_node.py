#!/usr/bin/env python3
import sys
from typing import Literal, Optional
import rclpy
from time import time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
from eel_interfaces.msg import TankStatus

from ..utils.constants import SIMULATE_PARAM
from ..utils.utils import clamp
from .tank_configs import get_tank_config

from ..utils.pid_controller import PidController
from .tank_utils.factory import create_hardware_tank, create_sim_tank

TANK_FILL_TIME_S = 22
# change depending on how big of an error we accept
TARGET_TOLERANCE = 0.02

LEVEL_FLOOR = 0.0
LEVEL_CEILING = 1.0

# TANK_RANGE_MM = TANK_CEILING_MM - TANK_FLOOR_MM
# TANK_FILL_VELOCITY_MMPS = TANK_RANGE_MM / TANK_FILL_TIME_S

# Minimum update frequency should be higher than tank fill velocity divided by target tolerance.
# Otherwise we might miss the target, since the pump has rushed past the target area before checking again.
# Example:
# tank floor: 30 mm
# tank ceiling: 65 mm
# tank fill time: 22 s
# target tolerance: 0.03 (3%)
# tank range: ceiling - floor = 65 - 30 = 35 mm
# fill velocity: range / fill time = 35 / 22 = 1.6 mm/s
# tolerance in mm: tolerance * range = 0.03 * 35 = 1.05 mm
# This means that 1 update per second is not enough, since it will run 1.6 mm in a second, and therefore
# miss the tolerance span of 1.05 mm.
# Minimum update frequency: velocity / tolerance = 1.6 / 1.05 = 1.5 hz
# UPDATE_FREQUENCY = 10 should therefore be plenty.
UPDATE_FREQUENCY = 10


TargetStatus = Literal[
    "target_reached", "ceiling_reached", "floor_reached", "no_target", "adjusting"
]


# TODO:
# '<=' not supported between instances of 'float' and 'NoneType'
def is_within_accepted_target_boundaries(
    current_level: Optional[float], target_level: Optional[float]
) -> bool:
    if current_level is None or target_level is None:
        return False
    low_threshold = target_level - (TARGET_TOLERANCE / 2)
    high_threshold = target_level + (TARGET_TOLERANCE / 2)
    is_within_target = low_threshold <= current_level <= high_threshold
    return is_within_target


def is_above_target(current_level: float, target_level: float) -> bool:
    high_threshold = target_level + (TARGET_TOLERANCE / 2)
    return current_level > high_threshold


def is_below_target(current_level: float, target_level: float) -> bool:
    low_threshold = target_level - (TARGET_TOLERANCE / 2)
    return current_level < low_threshold


def is_at_floor(current_level: float) -> bool:
    return current_level <= (LEVEL_FLOOR + TARGET_TOLERANCE)


def is_at_ceiling(current_level: float) -> bool:
    return current_level >= (LEVEL_CEILING - TARGET_TOLERANCE)


def get_level_velocity(
    level: float,
    previous_level: Optional[float],
    now: float,
    previous_level_at: Optional[float],
) -> float:
    if previous_level is None or previous_level_at is None:
        return 0.0
    level_delta = level - previous_level
    time_delta = now - previous_level_at
    velocity = level_delta / time_delta
    return velocity


# NOTE: example usage
# ros2 run eel tank --ros-args -p simulate:=false -p tank_type:=front
# ros2 run eel tank --ros-args -p simulate:=true -p tank_type:=rear


class RunningAverage:
    def __init__(self, size):
        self.size = size
        self.samples = [0.0] * size
        self.index = 0

    def add_sample(self, value):
        self.samples[self.index] = value
        self.index = (self.index + 1) % self.size

    def get_average(self):
        return sum(self.samples) / self.size


class TankNode(Node):
    def __init__(self, node_name="tank_node"):
        super().__init__(node_name, parameter_overrides=[])
        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter("tank_type", Parameter.Type.STRING)
        tank_type = self.get_parameter("tank_type").get_parameter_value().string_value
        if tank_type not in ["front", "rear"]:
            raise ValueError(f"tank_type must be 'front' or 'rear', got: {tank_type=}")

        self.config = get_tank_config(tank_type)

        should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)

        self.is_autocorrecting: bool = False
        self.target_level: Optional[float] = None

        # only used for passing information to frontend
        self.target_status: TargetStatus = "no_target"

        self.current_level: Optional[float] = None
        self.current_velocity: float = float()
        self.previous_level: Optional[float] = None
        self.previous_level_at: Optional[float] = None

        self.sample_index = 0
        self.sample_size = 10
        self.level_samples = [0.0 for _ in range(self.sample_size)]

        self.clamp_value = 0.0
        self.clamp_max_value = 1.0

        self.running_average = RunningAverage(10)

        # float Kp = 0.2;
        # float Ki = 0.05;
        # float Kd = 0.1;

        # GUNNAR
        # self.tank_motor_pid = PidController(0.0, kP=2.0, kI=0.1, kD=0.75)
        # self.tank_motor_pid = PidController(0.0, kP=4.0, kI=0.2, kD=1.2)
        # self.tank_motor_pid = PidController(0.0, kP=8.0, kI=0.5, kD=2.5)
        pid_config = {"kP": 8.0, "kI": 0.5, "kD": 2.5}

        # DAVID
        # self.tank_motor_pid = PidController(0.0, kP=1.0, kI=0.2, kD=0.75)
        # self.tank_motor_pid = PidController(0.0, kP=0.0, kI=0.3, kD=1.2)
        # self.tank_motor_pid = PidController(0.0, kP=0.0, kI=0.35, kD=2.5)

        # ADAM
        # self.tank_motor_pid = PidController(0.0, kP=4.0, kI=0.0, kD=1.2)
        # self.tank_motor_pid = PidController(0.0, kP=8.0, kI=0.0, kD=2.0)

        self.tank_motor_pid = PidController(
            0.0, pid_config["kP"], pid_config["kI"], pid_config["kD"]
        )

        self.level_cmd_subscription = self.create_subscription(
            Float32, self.config["cmd_topic"], self.handle_tank_cmd, 10
        )
        self.publisher = self.create_publisher(
            TankStatus, self.config["status_topic"], 10
        )

        if should_simulate:
            self.tank = create_sim_tank()
        else:
            self.tank = create_hardware_tank(
                self.config["motor_pin"],
                self.config["direction_pin"],
                self.config["tank_floor_value"],
                self.config["tank_ceiling_value"],
                self.config["distance_sensor_channel"],
            )

        self.check_target_updater = self.create_timer(
            1.0 / UPDATE_FREQUENCY, self.target_loop
        )

        self.get_logger().info(
            f"{should_simulate=} Tank node started with {self.config=} {UPDATE_FREQUENCY=} {pid_config=}"
        )

    def stop_checking_against_target(self):
        self.target_level = None
        self.is_autocorrecting = False

    def handle_tank_cmd(self, msg: Float32):
        requested_target_level = msg.data
        target_level = clamp(requested_target_level, LEVEL_FLOOR, LEVEL_CEILING)

        self.target_status = "adjusting"
        self.target_level = target_level

        # self.get_logger().info(f"Setting set point {self.target_level}")

        # NOTE: can we even do this here? since we're gonna call this often,
        # so the cumulative error is gonna be reset all the time.
        self.tank_motor_pid.reset_cumulative_error()

        reset_ramp = abs(self.target_level - self.tank_motor_pid.set_point) > 0.05
        if reset_ramp:
            self.clamp_value = 0.0

        self.tank_motor_pid.update_set_point(self.target_level)

    def publish_status(self, current_level: Optional[float]) -> None:
        if current_level is not None:
            msg = TankStatus()
            msg.current_level = float(current_level)
            msg.target_level = []
            if self.target_level:
                msg.target_level.append(self.target_level)
            msg.is_autocorrecting = self.is_autocorrecting
            msg.target_status = self.target_status
            self.publisher.publish(msg)

    def target_loop(self) -> None:
        self.current_level = self.tank.get_level()

        self.running_average.add_sample(self.current_level)

        level_average = self.running_average.get_average()

        # TODO: maybe publish both average and momentary level
        self.publish_status(self.current_level)

        if self.target_level is None:
            return

        level_error = abs(level_average - self.target_level)

        if level_error < 0.01:
            self.tank.stop()
            self.tank_motor_pid.reset_cumulative_error()
        else:
            if self.clamp_value <= self.clamp_max_value:
                self.clamp_value += 0.05
            pid_value = self.tank_motor_pid.compute(level_average)
            if abs(pid_value) > 1.0:
                self.tank_motor_pid.reset_cumulative_error()

            next_value = clamp(pid_value, -self.clamp_value, self.clamp_value)

            self.tank.set_speed(next_value)

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down")
        self.stop_checking_against_target()
        self.tank.stop()


def check_existing_nodes(target_node_name: str) -> bool:
    import subprocess

    """Check if a node with the given name is already running"""
    try:
        result = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=3
        )
        if result.returncode == 0:
            running_nodes = result.stdout.strip().split("\n")
            # Node names in the list have leading '/'
            full_node_name = f"/{target_node_name}"
            return full_node_name in running_nodes
    except (
        subprocess.TimeoutExpired,
        subprocess.CalledProcessError,
        FileNotFoundError,
    ):
        # ros2 CLI not available or failed - assume no conflict
        return False
    return False


def main(args=None):
    rclpy.init(args=args)

    # Parse tank_type from args to create appropriate node name.
    # A bit hacky, but helpful to see which tank is logged.
    tank_type = "unknown"
    for i, arg in enumerate(sys.argv):
        if "tank_type:=" in arg:
            tank_type = arg.split("tank_type:=")[1]
            break

    from rclpy.logging import get_logger

    logger = get_logger(__name__)

    node_name = f"{tank_type}_tank"
    if check_existing_nodes(node_name):
        logger.warn(
            f"WARNING: Node '{node_name}' appears to already be running! This can cause unexpected behaviour."
        )

    node = TankNode(node_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
