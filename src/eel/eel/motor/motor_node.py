#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .motor_sim import MotorSimulator
from ..utils.topics import MOTOR_CMD
from ..utils.constants import SIMULATE_PARAM
from ..utils.utils import clamp


class Motor(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.motor_subscription = self.create_subscription(
            Float32, MOTOR_CMD, self.handle_motor_msg, 10
        )

        if self.should_simulate:
            simulator = MotorSimulator()
            self.stop = simulator.stop
            self.forward = simulator.forward
            self.backward = simulator.backward
        else:
            from .motor_control import MotorControl

            motor_control = MotorControl()
            self.stop = motor_control.stop
            self.forward = motor_control.forward
            self.backward = motor_control.backward

        self.get_logger().info(
            "{}Motor node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def handle_motor_msg(self, msg):
        motor_value = clamp(msg.data, -1, 1)
        if motor_value == 0:
            self.stop()
        elif motor_value > 0:
            self.forward(signal=motor_value)
        elif motor_value < 0:
            self.backward(signal=abs(motor_value))


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
