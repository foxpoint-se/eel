#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
from eel_interfaces.msg import TankStatus

from ..utils.constants import (
    SIMULATE_PARAM,
    MOTOR_PIN_PARAM,
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_PIN_PARAM,
    CMD_TOPIC_PARAM,
    STATUS_TOPIC_PARAM,
    TANK_FLOOR_VALUE_PARAM,
    TANK_CEILING_VALUE_PARAM,
)
from ..utils.utils import clamp
from .pump_motor_simulator import PumpMotorControlSimulator
from .distance_sensor_simulator import DistanceSensorSimulator

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

DISTANCE_SENSOR_ERROR = "distance_sensor_error"
TARGET_REACHED = "target_reached"
CEILING_REACHED = "ceiling_reached"
FLOOR_REACHED = "floor_reached"
NO_TARGET = "no_target"
ADJUSTING = "adjusting"


def is_within_accepted_target_boundaries(current_level, target_level):
    low_threshold = target_level - (TARGET_TOLERANCE / 2)
    high_threshold = target_level + (TARGET_TOLERANCE / 2)
    is_within_target = low_threshold <= current_level <= high_threshold
    return is_within_target


def is_above_target(current_level, target_level):
    high_threshold = target_level + (TARGET_TOLERANCE / 2)
    return current_level > high_threshold


def is_below_target(current_level, target_level):
    low_threshold = target_level - (TARGET_TOLERANCE / 2)
    return current_level < low_threshold


def is_at_floor(current_level):
    return current_level <= (LEVEL_FLOOR + TARGET_TOLERANCE)


def is_at_ceiling(current_level):
    return current_level >= (LEVEL_CEILING - TARGET_TOLERANCE)


def should_fill(current_level, target_level):
    return current_level < target_level


def should_empty(current_level, target_level):
    return current_level > target_level


# NOTE: example usage
# OLD
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_rear/cmd -p status_topic:=/tank_rear/status -p motor_pin:=24 -p direction_pin:=25 -p tank_floor_mm:=15 -p tank_ceiling_mm:=63 -p xshut_pin_param:=21
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_front/cmd -p status_topic:=/tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p tank_floor_mm:=15 -p tank_ceiling_mm:=63 -p xshut_pin_param:=21
# NEW
# ros2 run eel tank --ros-args -p simulate:=False -p cmd_topic:=/tank_front/cmd -p status_topic:=/tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p tank_floor_value:=4736 -p tank_ceiling_value:=18256 -p distance_sensor_pin:=1


class TankNode(Node):
    def __init__(self):
        super().__init__("tank_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(CMD_TOPIC_PARAM)
        self.declare_parameter(STATUS_TOPIC_PARAM)
        self.declare_parameter(MOTOR_PIN_PARAM, -1)
        self.declare_parameter(DIRECTION_PIN_PARAM, -1)
        self.declare_parameter(DISTANCE_SENSOR_PIN_PARAM, -1)
        self.declare_parameter(TANK_FLOOR_VALUE_PARAM)
        self.declare_parameter(TANK_CEILING_VALUE_PARAM)

        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.motor_pin = int(self.get_parameter(MOTOR_PIN_PARAM).value)
        self.direction_pin = int(self.get_parameter(DIRECTION_PIN_PARAM).value)
        self.distance_sensor_pin = int(
            self.get_parameter(DISTANCE_SENSOR_PIN_PARAM).value
        )
        self.floor_value = int(self.get_parameter(TANK_FLOOR_VALUE_PARAM).value)
        self.ceiling_value = int(self.get_parameter(TANK_CEILING_VALUE_PARAM).value)

        self.is_autocorrecting = False
        self.target_level = None
        self.target_status = NO_TARGET  # only used for passing information to frontend
        self.current_level = None
        self.current_range = None

        self.cmd_topic = self.get_parameter(CMD_TOPIC_PARAM).value
        self.status_topic = self.get_parameter(STATUS_TOPIC_PARAM).value
        if not self.cmd_topic or not self.status_topic:
            raise TypeError(
                "Missing topic arguments ({}, {})".format(
                    CMD_TOPIC_PARAM, STATUS_TOPIC_PARAM
                )
            )

        self.debug_topic = "{}_debug".format(self.status_topic)

        self.level_cmd_subscription = self.create_subscription(
            Float32, self.cmd_topic, self.handle_tank_cmd, 10
        )
        self.publisher = self.create_publisher(TankStatus, self.status_topic, 10)
        self.debug_publisher = self.create_publisher(Float32, self.debug_topic, 10)

        if self.should_simulate:
            self.pump_motor_control = PumpMotorControlSimulator()
            self.distance_sensor = DistanceSensorSimulator(
                initial_measurement_percent=1.0,
                update_frequency_hz=UPDATE_FREQUENCY,
                create_timer=self.create_timer,
                get_is_motor_filling_up=self.pump_motor_control.get_is_filling_up,
                get_is_motor_emptying=self.pump_motor_control.get_is_emptying,
            )
        else:
            from .pump_motor_control import PumpMotorControl
            from .distance_sensor_potentiometer import DistanceSensorPotentiometer

            self.pump_motor_control = PumpMotorControl(
                motor_pin=self.motor_pin, direction_pin=self.direction_pin
            )

            self.distance_sensor = DistanceSensorPotentiometer(
                floor=self.floor_value,
                ceiling=self.ceiling_value,
                pin=self.distance_sensor_pin,
            )

        self.check_target_updater = self.create_timer(
            1.0 / UPDATE_FREQUENCY, self.target_loop
        )

        self.get_logger().info(
            "{}Tank node started. Motor pin: {}, Direction pin: {}, Distance sensor pin: {}, Update frequency: {}, Range: {} - {} mm, CMD topic: {}, status topic: {}".format(
                "SIMULATE " if self.should_simulate else "",
                self.motor_pin,
                self.direction_pin,
                self.distance_sensor_pin,
                UPDATE_FREQUENCY,
                self.floor_value,
                self.ceiling_value,
                self.cmd_topic,
                self.status_topic,
            )
        )

    def start_checking_against_target(self, target):
        self.target_level = target
        self.is_autocorrecting = True

    def stop_checking_against_target(self):
        self.target_level = None
        self.is_autocorrecting = False

    def handle_tank_cmd(self, msg):
        requested_target_level = msg.data
        current_level = self.get_level()
        target_level = clamp(requested_target_level, LEVEL_FLOOR, LEVEL_CEILING)

        if not is_within_accepted_target_boundaries(current_level, target_level):
            self.start_checking_against_target(target_level)
            self.start_motor_towards_target(current_level, target_level)
            self.target_status = ADJUSTING

    def start_motor_towards_target(self, current_level, target_level):
        if current_level > target_level:
            self.pump_motor_control.empty()
        else:
            self.pump_motor_control.fill()

    def publish_status(self, current_level):
        msg = TankStatus()
        msg.current_level = current_level
        msg.target_level = []
        if self.target_level:
            msg.target_level.append(self.target_level)
        msg.is_autocorrecting = self.is_autocorrecting
        msg.target_status = self.target_status
        self.publisher.publish(msg)

        msg2 = Float32()
        msg2.data = float(self.current_range or 0.0)
        self.debug_publisher.publish(msg2)

    def target_loop(self):
        current_level = self.get_level()
        if (
            self.target_level is not None
            and current_level
            and self.is_autocorrecting is True
        ):
            self.check_against_target(current_level, self.target_level)

        self.publish_status(current_level)

    def check_against_target(self, current_level, target_level):
        if is_within_accepted_target_boundaries(current_level, target_level):
            self.pump_motor_control.stop()
            self.stop_checking_against_target()
            self.target_status = TARGET_REACHED

        # Do we need these safety checks? Will they ever happen?
        # I guess that they could if the target check frequency is to low, so that the target then is missed.
        if is_at_floor(current_level) and self.pump_motor_control.get_is_emptying():
            self.pump_motor_control.stop()
            self.stop_checking_against_target()
            self.target_status = FLOOR_REACHED

        if is_at_ceiling(current_level) and self.pump_motor_control.get_is_filling_up():
            self.pump_motor_control.stop()
            self.stop_checking_against_target()
            self.target_status = CEILING_REACHED

        if self.pump_motor_control.get_is_filling_up() and is_above_target(
            current_level, target_level
        ):
            self.pump_motor_control.empty()

        if self.pump_motor_control.get_is_emptying() and is_below_target(
            current_level, target_level
        ):
            self.pump_motor_control.fill()

    def get_level(self):
        distance_level = self.distance_sensor.get_level()
        level_filled = 1 - distance_level
        return level_filled


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
        node.pump_motor_control.stop()
        node.stop_checking_against_target()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
