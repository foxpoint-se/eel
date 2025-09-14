#!/usr/bin/env python3
import sys
from typing import Literal, Optional, Union
import rclpy
from time import time
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from eel_interfaces.msg import TankStatus

from ..utils.constants import (
    SIMULATE_PARAM,
    MOTOR_PIN_PARAM,
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_CHANNEL_PARAM,
    CMD_TOPIC_PARAM,
    STATUS_TOPIC_PARAM,
    TANK_FLOOR_VALUE_PARAM,
    TANK_CEILING_VALUE_PARAM,
)
from ..utils.utils import clamp
from .tank_utils.create_tank import create_tank
from ..utils.pid_controller import PidController 

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
# OLD
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_rear/cmd -p status_topic:=/tank_rear/status -p motor_pin:=24 -p direction_pin:=25 -p tank_floor_mm:=15 -p tank_ceiling_mm:=63 -p xshut_pin_param:=21
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_front/cmd -p status_topic:=/tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p tank_floor_mm:=15 -p tank_ceiling_mm:=63 -p xshut_pin_param:=21
# NEW
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_front/cmd -p status_topic:=/tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p tank_floor_value:=4736 -p tank_ceiling_value:=18256 -p distance_sensor_channel:=1


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

# Example usage:
# avg = RunningAverage(5)
# for val in [1, 2, 3, 4, 5, 6]:
#     avg.add_sample(val)
#     print(avg.get_average())

class TankNode(Node):
    def __init__(self):
        super().__init__("tank_node", parameter_overrides=[])
        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(CMD_TOPIC_PARAM)
        self.declare_parameter(STATUS_TOPIC_PARAM)
        self.declare_parameter(MOTOR_PIN_PARAM, -1)
        self.declare_parameter(DIRECTION_PIN_PARAM, -1)
        self.declare_parameter(DISTANCE_SENSOR_CHANNEL_PARAM, -1)
        self.declare_parameter(TANK_FLOOR_VALUE_PARAM)
        self.declare_parameter(TANK_CEILING_VALUE_PARAM)

        should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)
        motor_pin = int(
            self.get_parameter(MOTOR_PIN_PARAM).get_parameter_value().integer_value
        )
        direction_pin = int(
            self.get_parameter(DIRECTION_PIN_PARAM).get_parameter_value().integer_value
        )
        distance_sensor_channel = int(
            self.get_parameter(DISTANCE_SENSOR_CHANNEL_PARAM)
            .get_parameter_value()
            .integer_value
        )
        floor_value = float(
            self.get_parameter(TANK_FLOOR_VALUE_PARAM)
            .get_parameter_value()
            .double_value
        )
        ceiling_value = float(
            self.get_parameter(TANK_CEILING_VALUE_PARAM)
            .get_parameter_value()
            .double_value
        )

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

        self.tank_motor_pid = PidController(0.0, kP=0.05, kI=0.0, kD=0.1)

        cmd_topic = str(
            self.get_parameter(CMD_TOPIC_PARAM).get_parameter_value().string_value
        )
        status_topic = str(
            self.get_parameter(STATUS_TOPIC_PARAM).get_parameter_value().string_value
        )
        if not cmd_topic or not status_topic:
            raise TypeError(
                "Missing topic arguments ({}, {})".format(
                    CMD_TOPIC_PARAM, STATUS_TOPIC_PARAM
                )
            )

        self.level_cmd_subscription = self.create_subscription(
            Float32, cmd_topic, self.handle_tank_cmd, 10
        )
        self.publisher = self.create_publisher(TankStatus, status_topic, 10)

        self.tank = create_tank(
            simulate=should_simulate,
            motor_pin=motor_pin,
            direction_pin=direction_pin,
            floor=floor_value,
            ceiling=ceiling_value,
            channel=distance_sensor_channel,
        )

        self.check_target_updater = self.create_timer(
            1.0 / UPDATE_FREQUENCY, self.target_loop
        )

        self.get_logger().info(
            "{}Tank node started. Motor pin: {}, Direction pin: {}, Distance sensor channel: {}, Update frequency: {}, Range: {} - {} mm, CMD topic: {}, status topic: {}".format(
                "SIMULATE " if should_simulate else "",
                motor_pin,
                direction_pin,
                distance_sensor_channel,
                UPDATE_FREQUENCY,
                floor_value,
                ceiling_value,
                cmd_topic,
                status_topic,
            )
        )


    def stop_checking_against_target(self):
        self.target_level = None
        self.is_autocorrecting = False

    def handle_tank_cmd(self, msg: Float32):
        requested_target_level = msg.data
        current_level = self.tank.get_level()
        target_level = clamp(requested_target_level, LEVEL_FLOOR, LEVEL_CEILING)


        if not is_within_accepted_target_boundaries(current_level, target_level):
            self.target_status = "adjusting"
            self.target_level = target_level
            
            self.get_logger().info(f"Setting set point {self.target_level}")
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

        if self.target_level and abs(self.current_level - self.target_level) < 0.01:
            self.tank.stop()
            self.tank_motor_pid.reset_cumulative_error()
        else:
            self.level_samples[self.sample_index] = self.current_level
            self.sample_index = (self.sample_index + 1) % self.sample_size
            level_average = sum(self.level_samples) / self.sample_size

            if (
                self.target_level is not None
                and self.current_level
            ):
                next_value = self.tank_motor_pid.compute(level_average)

                clamped_next_value = clamp(next_value, -1.0, 1.0)
                
                self.tank.run_motor(clamped_next_value)

        self.publish_status(self.current_level)


    def shutdown(self) -> None:
        self.get_logger().info("Shutting down")
        self.stop_checking_against_target()
        self.tank.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TankNode()

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
