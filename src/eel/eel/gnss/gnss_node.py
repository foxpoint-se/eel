"""GNSS logic: raw NavSatFix → Coordinate on gnss/status."""

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from eel_interfaces.msg import Coordinate

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import GNSS_FIX, GNSS_STATUS
from .gnss_math import is_valid_gnss_latlon


class GnssNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss")
        self.create_subscription(NavSatFix, GNSS_FIX, self._handle_fix, 10)
        self._pub = self.create_publisher(Coordinate, GNSS_STATUS, 10)
        self.get_logger().info(f"GNSS started (listening on {GNSS_FIX})")

    def _handle_fix(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if not is_valid_gnss_latlon(lat, lon):
            return

        out = Coordinate()
        out.lat = lat
        out.lon = lon
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GnssNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
