#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from eel_interfaces.msg import TankStatus
from ..utils.topics import TANK_CMD, TANK_STATUS
from ..utils.constants import (
    SIMULATE_PARAM,
    MOTOR_PIN_PARAM,
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_ADDRESS_PARAM,
)
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
# change depending on how what we think is a good frequency
WANTED_TARGET_CHECK_FREQUENCY = 5

LEVEL_FLOOR = 0.0
LEVEL_CEILING = 1.0

TANK_RANGE_MM = TANK_CEILING_MM - TANK_FLOOR_MM
TANK_FILL_VELOCITY_MMPS = TANK_RANGE_MM / TANK_FILL_TIME_S
TARGET_TOLERANCE_MM = TARGET_TOLERANCE * TANK_RANGE_MM

MINIMUM_UPDATE_FREQUENCY = TANK_FILL_VELOCITY_MMPS / TARGET_TOLERANCE_MM

TARGET_CHECK_FREQUENCY = (
    WANTED_TARGET_CHECK_FREQUENCY
    if WANTED_TARGET_CHECK_FREQUENCY > MINIMUM_UPDATE_FREQUENCY
    else MINIMUM_UPDATE_FREQUENCY
)

STATUS_PUBLISH_FREQUENCY = 1

S_TO_NS = 1000 * 1000 * 1000


TARGET_REACHED = "target_reached"
CEILING_REACHED = "ceiling_reached"
FLOOR_REACHED = "floor_reached"
NO_TARGET = "no_target"
ADJUSTING = "adjusting"


def frequency_to_period_in_ns(frequency):
    return (1 / frequency) * S_TO_NS


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

        self.level_cmd_subscription = self.create_subscription(
            Float32, TANK_CMD, self.handle_tank_cmd, 10
        )
        self.publisher = self.create_publisher(TankStatus, TANK_STATUS, 10)

        if self.should_simulate:
            self.pump_motor_control = PumpMotorControlSimulator()
            self.distance_sensor = DistanceSensorSimulator(
                initial_measurement_mm=TANK_FLOOR_MM,
                update_frequency_hz=TARGET_CHECK_FREQUENCY,
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
            "{}Tank node started. Motor pin: {}, Direction pin: {}, Distance sensor address: {}, Target check frequency: {}, Status publish frequency: {}".format(
                "SIMULATE " if self.should_simulate else "",
                self.motor_pin,
                self.direction_pin,
                self.distance_sensor_address,
                TARGET_CHECK_FREQUENCY,
                STATUS_PUBLISH_FREQUENCY,
            )
        )

    def should_check_against_level(self):
        return self.target_level is not None and self.is_autocorrecting is True

    def init_timer(self):
        frequency = (
            TARGET_CHECK_FREQUENCY
            if self.should_check_against_level()
            else STATUS_PUBLISH_FREQUENCY
        )
        return self.create_timer(1.0 / frequency, self.loop)

    def start_checking_against_target(self, target):
        self.target_level = target
        self.is_autocorrecting = True
        self.level_updater.timer_period_ns = frequency_to_period_in_ns(
            TARGET_CHECK_FREQUENCY
        )

    def stop_checking_against_target(self):
        self.target_level = None
        self.is_autocorrecting = False
        self.level_updater.timer_period_ns = frequency_to_period_in_ns(
            STATUS_PUBLISH_FREQUENCY
        )

    def handle_tank_cmd(self, msg):
        target_level = msg.data
        current_level = self.get_level()

        if is_at_ceiling(current_level) and should_fill(current_level, target_level):
            self.stop_checking_against_target()
            self.target_status = CEILING_REACHED
            return
        if is_at_floor(current_level) and should_empty(current_level, target_level):
            self.stop_checking_against_target()
            self.target_status = FLOOR_REACHED
            return

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
