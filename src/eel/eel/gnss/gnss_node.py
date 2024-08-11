#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import Coordinate
from .gnss_sensor import GnssSensor
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import GNSS_STATUS


class GNSS(Node):
    def __init__(self):
        super().__init__("gnss_node", parameter_overrides=[])

        # The gnss sensor delivers new positions every second.
        # To not get sampling errors, one should sample at least twice the frequency
        # of the "real world" values. Two hertz is therefore good enough in this case.
        self.update_frequency_hz = 2

        self.publisher = self.create_publisher(Coordinate, GNSS_STATUS, 10)
        self.declare_parameter(SIMULATE_PARAM, False)

        # At the time of writing, Ålen uses /dev/ttyUSB1 (which is why it's the default)
        # and Tvålen uses /dev/ttyUSB0 for some reason.
        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )

        sensor = GnssSensor(serial_port=serial_port)
        self.get_current_position = sensor.get_current_position

        self.poller = self.create_timer(1.0 / self.update_frequency_hz, self.publish)
        self.get_logger().info("GNSS node started.")

    def publish(self):
        lat, lon = self.get_current_position()

        if isinstance(lat, float) and isinstance(lon, float):
            if abs(lat) > 0.1 and abs(lon) > 0.1:
                msg = Coordinate()
                msg.lat = lat
                msg.lon = lon

                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
