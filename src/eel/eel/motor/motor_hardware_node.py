"""Pure motor I/O: motor/setpoint → GPIO ESC."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MOTOR_SETPOINT
from .motor_control import MotorControl


class MotorHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_hardware")
        self._motor = MotorControl()
        self.create_subscription(Float32, MOTOR_SETPOINT, self._handle_setpoint, 10)
        self.get_logger().info("Motor hardware started (listening on motor/setpoint)")

    def shutdown(self) -> None:
        self._motor.close()

    def _handle_setpoint(self, msg: Float32) -> None:
        value = float(msg.data)
        if value == 0.0:
            self._motor.stop()
        elif value > 0.0:
            self._motor.forward(signal=value)
        else:
            self._motor.backward(signal=abs(value))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotorHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
