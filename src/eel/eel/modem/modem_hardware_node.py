"""Pure modem I/O: serial AT → modem/raw."""

from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import ModemRaw

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import MODEM_RAW
from .modem_sensor import ModemSensor

PUBLISH_PERIOD_S = 2.0


class ModemHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("modem_hardware")
        self._sensor = ModemSensor()
        reg_status = self._sensor.get_registration_status()
        signal_strength = self._sensor.get_received_signal_strength_indicator()
        if reg_status is None or signal_strength is None:
            raise Exception("Could not start modem hardware. Not getting registration status and/or signal strength.")

        self._pub = self.create_publisher(ModemRaw, MODEM_RAW, 10)
        self.create_timer(PUBLISH_PERIOD_S, self._publish_raw)
        self.get_logger().info("Modem hardware started (publishing modem/raw)")

    def _publish_raw(self) -> None:
        reg_status = self._sensor.get_registration_status()
        signal_strength = self._sensor.get_received_signal_strength_indicator()
        if reg_status is None or signal_strength is None:
            return
        out = ModemRaw()
        out.reg_status = reg_status
        out.signal_strength = signal_strength
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ModemHardwareNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
