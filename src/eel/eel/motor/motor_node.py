#!/usr/bin/env python3
from time import monotonic
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ..utils.actuator_bounds import bounded_unit_cmd
from ..utils.constants import SIMULATE_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MOTOR_CMD
from .motor_sim import MotorSimulator
from .motor_watchdog import command_is_stale, require_positive_cmd_timeout

MOTOR_CMD_TIMEOUT_PARAM = "cmd_timeout_s"
DEFAULT_MOTOR_CMD_TIMEOUT_S = 1.0
WATCHDOG_CHECK_PERIOD_S = 0.2


class Motor(Node):
    def __init__(self) -> None:
        super().__init__("motor_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.declare_parameter(MOTOR_CMD_TIMEOUT_PARAM, DEFAULT_MOTOR_CMD_TIMEOUT_S)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.cmd_timeout_s = require_positive_cmd_timeout(float(self.get_parameter(MOTOR_CMD_TIMEOUT_PARAM).value))

        self.motor_subscription = self.create_subscription(Float32, MOTOR_CMD, self.handle_motor_msg, 10)
        self._last_cmd_at: float | None = None

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

        self.create_timer(WATCHDOG_CHECK_PERIOD_S, self._watchdog_tick)

        self.get_logger().info(
            "{}Motor node started (cmd_timeout_s={}).".format(
                "SIMULATE " if self.should_simulate else "",
                self.cmd_timeout_s,
            )
        )

    def shutdown(self) -> None:
        self.stop()
        if self._motor_control is not None:
            self._motor_control.close()

    def handle_motor_msg(self, msg: Float32) -> None:
        motor_value = bounded_unit_cmd(msg.data)
        if motor_value is None:
            self.get_logger().warning(f"Rejecting invalid motor cmd {msg.data}")
            return
        if motor_value == 0:
            self.stop()
            self._last_cmd_at = None
            return
        self._last_cmd_at = monotonic()
        if motor_value > 0:
            self.forward(signal=motor_value)
        else:
            self.backward(signal=abs(motor_value))

    def _watchdog_tick(self) -> None:
        if command_is_stale(self._last_cmd_at, monotonic(), self.cmd_timeout_s):
            self.get_logger().warning("motor/cmd stale; stopping motor")
            self.stop()
            self._last_cmd_at = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Motor()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
