#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ..utils.constants import SIMULATE_PARAM, MOTOR_PIN_PARAM, DIRECTION_PIN_PARAM
from .pump_motor_simulator import PumpMotorControlSimulator


class TankNode(Node):
    def __init__(self):
        super().__init__("tank_node")

        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(MOTOR_PIN_PARAM, -1)
        self.declare_parameter(DIRECTION_PIN_PARAM, -1)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.motor_pin = int(self.get_parameter(MOTOR_PIN_PARAM).value)
        self.direction_pin = int(self.get_parameter(DIRECTION_PIN_PARAM).value)

        if self.should_simulate:
            pump_motor_control = PumpMotorControlSimulator()
        if not self.should_simulate:
            from .pump_motor_control import PumpMotorControl

            pump_motor_control = PumpMotorControl(
                motor_pin=self.motor_pin, direction_pin=self.direction_pin
            )

        self.get_logger().info(
            "{}Tank node started. Motor pin: {}, Direction pin: {}".format(
                "SIMULATE " if self.should_simulate else "",
                self.motor_pin,
                self.direction_pin,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = TankNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
