#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from eel_interfaces.msg import TankStatus
from ..utils.constants import (
    SIMULATE_PARAM,
    MOTOR_PIN_PARAM,
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_ADDRESS_PARAM,
    CMD_TOPIC_PARAM,
    STATUS_TOPIC_PARAM,
)
from ..utils.utils import clamp
from .pump_motor_simulator import PumpMotorControlSimulator
from .distance_sensor_simulator import DistanceSensorSimulator

# change depending on measured values
TANK_FLOOR_MM = 30
# change depending on measured values
TANK_CEILING_MM = 65
# change depending on measured values
TANK_FILL_TIME_S = 22
# change depending on how big of an error we accept
TARGET_TOLERANCE = 0.03

LEVEL_FLOOR = 0.0
LEVEL_CEILING = 1.0

TANK_RANGE_MM = TANK_CEILING_MM - TANK_FLOOR_MM
TANK_FILL_VELOCITY_MMPS = TANK_RANGE_MM / TANK_FILL_TIME_S

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
# UPDATE_FREQUENCY = 5 should therefore be plenty.
UPDATE_FREQUENCY = 5

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


def is_at_floor(current_level):
    return current_level <= (LEVEL_FLOOR + TARGET_TOLERANCE)


def is_at_ceiling(current_level):
    return current_level >= (LEVEL_CEILING - TARGET_TOLERANCE)


def should_fill(current_level, target_level):
    return current_level < target_level


def should_empty(current_level, target_level):
    return current_level > target_level


class TankNode(Node):
    def __init__(self):
        super().__init__("tank_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(CMD_TOPIC_PARAM)
        self.declare_parameter(STATUS_TOPIC_PARAM)
        self.declare_parameter(MOTOR_PIN_PARAM, -1)
        self.declare_parameter(DIRECTION_PIN_PARAM, -1)
        self.declare_parameter(DISTANCE_SENSOR_ADDRESS_PARAM, -1)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.motor_pin = int(self.get_parameter(MOTOR_PIN_PARAM).value)
        self.direction_pin = int(self.get_parameter(DIRECTION_PIN_PARAM).value)
        self.distance_sensor_address = int(
            self.get_parameter(DISTANCE_SENSOR_ADDRESS_PARAM).value
        )
        self.is_autocorrecting = False
        self.target_level = None
        self.target_status = NO_TARGET  # only used for passing information to frontend

        self.cmd_topic = self.get_parameter(CMD_TOPIC_PARAM).value
        self.status_topic = self.get_parameter(STATUS_TOPIC_PARAM).value
        if not self.cmd_topic or not self.status_topic:
            raise TypeError(
                "Missing topic arguments ({}, {})".format(
                    CMD_TOPIC_PARAM, STATUS_TOPIC_PARAM
                )
            )

        self.level_cmd_subscription = self.create_subscription(
            Float32, self.cmd_topic, self.handle_tank_cmd, 10
        )
        self.publisher = self.create_publisher(TankStatus, self.status_topic, 10)

        if self.should_simulate:
            self.pump_motor_control = PumpMotorControlSimulator()
            self.distance_sensor = DistanceSensorSimulator(
                initial_measurement_mm=TANK_FLOOR_MM,
                update_frequency_hz=UPDATE_FREQUENCY,
                fill_velocity_mmps=TANK_FILL_VELOCITY_MMPS,
                create_timer=self.create_timer,
                get_is_motor_filling_up=self.pump_motor_control.get_is_filling_up,
                get_is_motor_emptying=self.pump_motor_control.get_is_emptying,
            )
        else:
            from .pump_motor_control import PumpMotorControl
            from .distance_sensor import DistanceSensor

            self.pump_motor_control = PumpMotorControl(
                motor_pin=self.motor_pin, direction_pin=self.direction_pin
            )

            self.distance_sensor = DistanceSensor(address=self.distance_sensor_address)

        self.level_updater = self.init_timer()

        self.get_logger().info(
            "{}Tank node started. Motor pin: {}, Direction pin: {}, Distance sensor address: {}, Update frequency: {}".format(
                "SIMULATE " if self.should_simulate else "",
                self.motor_pin,
                self.direction_pin,
                self.distance_sensor_address,
                UPDATE_FREQUENCY,
            )
        )

    def should_check_against_level(self):
        return self.target_level is not None and self.is_autocorrecting is True

    def init_timer(self):
        return self.create_timer(1.0 / UPDATE_FREQUENCY, self.loop)

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

    def loop(self):
        current_level = self.get_level()
        target_level = self.target_level
        if self.should_check_against_level():
            self.check_against_target(current_level, target_level)

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

    def get_level(self):
        range = self.distance_sensor.get_range()
        level = (range - TANK_FLOOR_MM) / (TANK_CEILING_MM - TANK_FLOOR_MM)
        return level


def main(args=None):
    rclpy.init(args=args)
    node = TankNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
