"""Reads the pressure sensor over serial and publishes depth in meters."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import PRESSURE_DEPTH_M
from .pressure_math import has_depth_sample
from .pressure_serial_driver import PressureSerialDriver

PUBLISH_HZ = 5.0


class PressureHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("pressure_hardware")
        self.declare_parameter("serial_port", Parameter.Type.STRING)
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value or None
        if not serial_port:
            raise ValueError("serial_port is required, e.g. --ros-args -p serial_port:=/dev/ttyUSB0")

        self._driver = PressureSerialDriver(serial_port)
        self._logged_calibration = False
        self._pub = self.create_publisher(Float32, PRESSURE_DEPTH_M, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_depth)
        self.get_logger().info(f"Waiting for first depth sample on {serial_port}")

    def _publish_depth(self) -> None:
        depth_m = self._driver.get_depth_m()
        if not has_depth_sample(depth_m):
            return
        if self._driver.is_calibrated and not self._logged_calibration:
            self.get_logger().info("Atmosphere offset set from first sample")
            self._logged_calibration = True

        msg = Float32()
        msg.data = depth_m
        self._pub.publish(msg)

    def close_driver(self) -> None:
        self._driver.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PressureHardwareNode()
    spin_node_until_shutdown(node, cleanup=node.close_driver)


if __name__ == "__main__":
    main()
