"""Pure leakage I/O: GPIO → leakage/raw."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import LEAKAGE_RAW
from .leakage_sensor import LeakageSensor

PUBLISH_HZ = 1.0


class LeakageHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("leakage_hardware")
        self._sensor = LeakageSensor()
        self._pub = self.create_publisher(Bool, LEAKAGE_RAW, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_raw)
        self.get_logger().info("Leakage hardware started (publishing leakage/raw)")

    def _publish_raw(self) -> None:
        msg = Bool()
        msg.data = bool(self._sensor.read_sensor())
        self._pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LeakageHardwareNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
