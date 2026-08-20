"""Pure LED I/O: led/setpoint → GPIO."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import LED_SETPOINT
from .led_control import LEDControl


class LedHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("led_hardware")
        self._led = LEDControl()
        self.create_subscription(Bool, LED_SETPOINT, self._handle_setpoint, 10)
        self.get_logger().info("LED hardware started (listening on led/setpoint)")

    def shutdown(self) -> None:
        self._led.close()

    def _handle_setpoint(self, msg: Bool) -> None:
        if msg.data:
            self._led.on()
        else:
            self._led.off()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LedHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
