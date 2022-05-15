#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ..utils.topics import TANK_CMD, TANK_STATUS
from ..utils.constants import (
    SIMULATE_PARAM,
    MOTOR_PIN_PARAM,
    DIRECTION_PIN_PARAM,
    DISTANCE_SENSOR_ADDRESS_PARAM,
)
from .pump_motor_simulator import PumpMotorControlSimulator
from .distance_sensor_simulator import DistanceSensorSimulator


class TankNode(Node):
    def __init__(self):
        super().__init__("tank_node")

        self.update_frequency = 5

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

        self.level_updater = self.create_timer(
            1.0 / self.update_frequency, self.check_against_target
        )

        self.level_cmd_subscription = self.create_subscription(
            Float32, TANK_CMD, self.set_target_level, 10
        )

        self.publisher = self.create_publisher(Float32, TANK_STATUS, 10)

        self.RANGE_FLOOR = 30
        self.RANGE_CEILING = 65
        self.ACCEPTED_OFFSET = 0.03

        self.current_level = 0.0
        self.target_level = 0.0

        if self.should_simulate:
            self.pump_motor_control = PumpMotorControlSimulator()
            self.distance_sensor = DistanceSensorSimulator(parent_node=self)
        if not self.should_simulate:
            from .pump_motor_control import PumpMotorControl
            from .distance_sensor import DistanceSensor

            self.pump_motor_control = PumpMotorControl(
                motor_pin=self.motor_pin, direction_pin=self.direction_pin
            )

            self.distance_sensor = DistanceSensor(address=self.distance_sensor_address)

        self.get_logger().info(
            "{}Tank node started. Motor pin: {}, Direction pin: {}, Distance sensor address: {}".format(
                "SIMULATE " if self.should_simulate else "",
                self.motor_pin,
                self.direction_pin,
                self.distance_sensor_address,
            )
        )

    def set_target_level(self, msg):
        self.target_level = msg.data
        current_level = self.get_level()
        if current_level > self.target_level:
            self.pump_motor_control.empty()
        else:
            self.pump_motor_control.fill()

    # TODO: fix so that we don't run empty or fill if we are already there
    def check_against_target(self):
        self.current_level = self.get_level()
        low_threshold = self.target_level - self.ACCEPTED_OFFSET
        high_threshold = self.target_level + self.ACCEPTED_OFFSET
        is_within_target = low_threshold < self.current_level < high_threshold
        has_reached_floor = self.current_level <= (0.0 + self.ACCEPTED_OFFSET)
        has_reached_ceiling = self.current_level >= (1.0 - self.ACCEPTED_OFFSET)
        if any([is_within_target, has_reached_ceiling, has_reached_floor]):
            self.pump_motor_control.stop()

        msg = Float32()
        msg.data = self.current_level
        self.publisher.publish(msg)

    def get_level(self):
        range = self.distance_sensor.get_range()
        level = (range - self.RANGE_FLOOR) / (self.RANGE_CEILING - self.RANGE_FLOOR)

        return level


def main(args=None):
    rclpy.init(args=args)
    node = TankNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
