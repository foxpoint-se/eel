"""Pure rudder I/O: rudder/setpoint → servo pulses (flip only; offset already applied)."""

from typing import Optional

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import RUDDER_SETPOINT
from .actuator.actuator import default_x_options, default_y_options
from .actuator.general_servo import RudderServo
from .actuator.types import ServoOptions


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
        self.create_subscription(Vector3, RUDDER_SETPOINT, self._handle_setpoint, 10)
        self.get_logger().info("Rudder hardware started (listening on rudder/setpoint)")

    def shutdown(self) -> None:
        self._x_servo.detach()
        self._y_servo.detach()

    def _handle_setpoint(self, msg: Vector3) -> None:
        self._x_servo.set_value(float(msg.x))
        self._y_servo.set_value(float(msg.y))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RudderHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
