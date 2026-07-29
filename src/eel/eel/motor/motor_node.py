#!/usr/bin/env python3
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.constants import SIMULATE_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MOTOR_CMD
from ..utils.utils import clamp
from .motor_sim import MotorSimulator


class Motor(Node):
    def __init__(self) -> None:
        super().__init__("motor_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.motor_subscription = self.create_subscription(Float32, MOTOR_CMD, self.handle_motor_msg, 10)

        stop: Callable[[], None]
        forward: Callable[[float], None]
        backward: Callable[[float], None]
        self._motor_control = None
        if self.should_simulate:
            simulator = MotorSimulator()
            stop = simulator.stop
            forward = simulator.forward
            backward = simulator.backward
        else:
            from .motor_control import MotorControl

            self._motor_control = MotorControl()
            stop = self._motor_control.stop
            forward = self._motor_control.forward
            backward = self._motor_control.backward

        self.stop = stop
        self.forward = forward
        self.backward = backward

        self.get_logger().info("{}Motor node started.".format("SIMULATE " if self.should_simulate else ""))

    def shutdown(self) -> None:
        self.stop()
        if self._motor_control is not None:
            self._motor_control.close()

    def handle_motor_msg(self, msg: Float32) -> None:
        motor_value = clamp(msg.data, -1, 1)
        if motor_value == 0:
            self.stop()
        elif motor_value > 0:
            self.forward(signal=motor_value)
        elif motor_value < 0:
            self.backward(signal=abs(motor_value))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Motor()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
