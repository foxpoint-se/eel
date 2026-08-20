"""Pure rudder I/O: rudder/setpoint → servo pulses (flip only; offset already applied)."""

from time import monotonic
from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from ..motor.motor_watchdog import command_is_stale, require_positive_cmd_timeout
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import RUDDER_SETPOINT
from .actuator.actuator import default_x_options, default_y_options
from .actuator.general_servo import RudderServo
from .actuator.types import ServoOptions

SETPOINT_TIMEOUT_PARAM = "setpoint_timeout_s"
DEFAULT_SETPOINT_TIMEOUT_S = 1.0
WATCHDOG_CHECK_PERIOD_S = 0.2


def _options_without_offset(base: ServoOptions) -> ServoOptions:
    return {
        "pin": base["pin"],
        "min_pulse_width": base["min_pulse_width"],
        "max_pulse_width": base["max_pulse_width"],
        "flip_direction": base["flip_direction"],
        "cap_min": base["cap_min"],
        "cap_max": base["cap_max"],
        "offset": 0.0,
    }


class RudderHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("rudder_hardware")
        pigpiod_host_parameter = "pigpiod_host"
        self.declare_parameter(pigpiod_host_parameter, "localhost")
        pigpiod_host = str(self.get_parameter(pigpiod_host_parameter).value)

        self._x_servo = RudderServo(
            options=_options_without_offset(default_x_options),
            pigpiod_host=pigpiod_host,
        )
        self._y_servo = RudderServo(
            options=_options_without_offset(default_y_options),
            pigpiod_host=pigpiod_host,
        )
        self.declare_parameter(SETPOINT_TIMEOUT_PARAM, DEFAULT_SETPOINT_TIMEOUT_S)
        self._setpoint_timeout_s = require_positive_cmd_timeout(float(self.get_parameter(SETPOINT_TIMEOUT_PARAM).value))
        self._last_setpoint_at: float | None = None
        self.create_subscription(Vector3, RUDDER_SETPOINT, self._handle_setpoint, 10)
        self.create_timer(WATCHDOG_CHECK_PERIOD_S, self._watchdog_tick)
        self.get_logger().info("Rudder hardware started (listening on rudder/setpoint)")

    def shutdown(self) -> None:
        self._x_servo.detach()
        self._y_servo.detach()

    def _handle_setpoint(self, msg: Vector3) -> None:
        self._last_setpoint_at = monotonic()
        self._x_servo.set_value(float(msg.x))
        self._y_servo.set_value(float(msg.y))

    def _watchdog_tick(self) -> None:
        if command_is_stale(self._last_setpoint_at, monotonic(), self._setpoint_timeout_s):
            self.get_logger().warning("rudder/setpoint stale; centering servos")
            self._last_setpoint_at = None
            self._x_servo.set_value(0.0)
            self._y_servo.set_value(0.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RudderHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
