"""Reads the GNSS chip over serial and publishes WGS84 as NavSatFix."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import GNSS_FIX
from .gnss_sensor import GnssSensor

PUBLISH_HZ = 2.0
FRAME_ID = "gnss_link"


class GnssHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("gnss_hardware")
        self.declare_parameter("serial_port", Parameter.Type.STRING)
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value or None
        if not serial_port:
            raise ValueError("serial_port is required, e.g. --ros-args -p serial_port:=/dev/ttyUSB0")

        self._sensor = GnssSensor(serial_port=serial_port)
        self._pub = self.create_publisher(NavSatFix, GNSS_FIX, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_fix)
        self.get_logger().info(f"Waiting for first GNSS fix on {serial_port}")

    def _publish_fix(self) -> None:
        lat, lon = self._sensor.get_current_position()
        if not isinstance(lat, float) or not isinstance(lon, float):
            return
        if abs(lat) <= 0.1 or abs(lon) <= 0.1:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = float("nan")
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GnssHardwareNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
