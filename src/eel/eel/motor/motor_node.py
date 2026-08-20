"""Motor logic: motor/cmd → motor/setpoint (bounds + watchdog)."""

from time import monotonic
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.actuator_bounds import bounded_unit_cmd
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MOTOR_CMD, MOTOR_SETPOINT
from .motor_watchdog import command_is_stale, require_positive_cmd_timeout

MOTOR_CMD_TIMEOUT_PARAM = "cmd_timeout_s"
DEFAULT_MOTOR_CMD_TIMEOUT_S = 1.0
WATCHDOG_CHECK_PERIOD_S = 0.2


class MotorNode(Node):
    def __init__(self) -> None:
        super().__init__("motor")
        self.declare_parameter(MOTOR_CMD_TIMEOUT_PARAM, DEFAULT_MOTOR_CMD_TIMEOUT_S)
        self.cmd_timeout_s = require_positive_cmd_timeout(float(self.get_parameter(MOTOR_CMD_TIMEOUT_PARAM).value))
        self._last_cmd_at: float | None = None
        self._setpoint_pub = self.create_publisher(Float32, MOTOR_SETPOINT, 10)
        self.create_subscription(Float32, MOTOR_CMD, self._handle_cmd, 10)
        self.create_timer(WATCHDOG_CHECK_PERIOD_S, self._watchdog_tick)
        self.get_logger().info(f"Motor started (cmd_timeout_s={self.cmd_timeout_s})")

    def _publish_setpoint(self, value: float) -> None:
        msg = Float32()
        msg.data = value
        self._setpoint_pub.publish(msg)

    def _handle_cmd(self, msg: Float32) -> None:
        motor_value = bounded_unit_cmd(msg.data)
        if motor_value is None:
            self.get_logger().warning(f"Rejecting invalid motor cmd {msg.data}")
            return
        if motor_value == 0:
            self._last_cmd_at = None
            self._publish_setpoint(0.0)
            return
        self._last_cmd_at = monotonic()
        self._publish_setpoint(motor_value)

    def _watchdog_tick(self) -> None:
        if command_is_stale(self._last_cmd_at, monotonic(), self.cmd_timeout_s):
            self.get_logger().warning("motor/cmd stale; stopping motor")
            self._last_cmd_at = None
            self._publish_setpoint(0.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotorNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
