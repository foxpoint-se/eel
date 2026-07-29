#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import Coordinate

from ..utils.constants import SIMULATE_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import GNSS_STATUS
from .gnss_sim import GnssSimulator, GnssSource


class GNSS(Node):
    def __init__(self) -> None:
        super().__init__("gnss_node", parameter_overrides=[])

        # The gnss sensor delivers new positions every second.
        # To not get sampling errors, one should sample at least twice the frequency
        # of the "real world" values. Two hertz is therefore good enough in this case.
        self.update_frequency_hz = 2

        self.publisher = self.create_publisher(Coordinate, GNSS_STATUS, 10)
        self.declare_parameter(SIMULATE_PARAM, False)
        should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)

        source: GnssSource
        if should_simulate:
            source = GnssSimulator()
        else:
            from .gnss_sensor import GnssSensor

            self.declare_parameter("serial_port", "/dev/ttyUSB1")
            serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
            source = GnssSensor(serial_port=serial_port)

        self.get_current_position = source.get_current_position

        self.poller = self.create_timer(1.0 / self.update_frequency_hz, self.publish)
        self.get_logger().info(f"{'SIMULATE ' if should_simulate else ''}GNSS node started.")

    def publish(self) -> None:
        lat, lon = self.get_current_position()

        if isinstance(lat, float) and isinstance(lon, float):
            if abs(lat) > 0.1 and abs(lon) > 0.1:
                msg = Coordinate()
                msg.lat = lat
                msg.lon = lon

                self.publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GNSS()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
