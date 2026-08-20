"""Pure motor I/O: motor/setpoint → GPIO ESC."""

from time import monotonic
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MOTOR_SETPOINT
from .motor_control import MotorControl
from .motor_watchdog import command_is_stale, require_positive_cmd_timeout

SETPOINT_TIMEOUT_PARAM = "setpoint_timeout_s"
DEFAULT_SETPOINT_TIMEOUT_S = 1.0
WATCHDOG_CHECK_PERIOD_S = 0.2


class MotorHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_hardware")
        self.declare_parameter(SETPOINT_TIMEOUT_PARAM, DEFAULT_SETPOINT_TIMEOUT_S)
        self._setpoint_timeout_s = require_positive_cmd_timeout(float(self.get_parameter(SETPOINT_TIMEOUT_PARAM).value))
        self._last_setpoint_at: float | None = None
        self._motor = MotorControl()
        self.create_subscription(Float32, MOTOR_SETPOINT, self._handle_setpoint, 10)
        self.create_timer(WATCHDOG_CHECK_PERIOD_S, self._watchdog_tick)
        self.get_logger().info(
            f"Motor hardware started (listening on motor/setpoint, setpoint_timeout_s={self._setpoint_timeout_s})"
        )

    def shutdown(self) -> None:
        self._motor.close()

    def _handle_setpoint(self, msg: Float32) -> None:
        value = float(msg.data)
        if value == 0.0:
            self._last_setpoint_at = None
            self._motor.stop()
            return
        self._last_setpoint_at = monotonic()
        if value > 0.0:
            self._motor.forward(signal=value)
        else:
            self._motor.backward(signal=abs(value))

    def _watchdog_tick(self) -> None:
        if command_is_stale(self._last_setpoint_at, monotonic(), self._setpoint_timeout_s):
            self.get_logger().warning("motor/setpoint stale; stopping motor")
            self._last_setpoint_at = None
            self._motor.stop()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotorHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
