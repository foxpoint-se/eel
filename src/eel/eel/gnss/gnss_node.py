#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import GNSS_STATUS


class GNSS(Node):
    def __init__(self):
        super().__init__("gnss_node")

        # hertz (updates per second), set to two since there is only one gnss msg per second
        self.update_frequency = 2

        self.publisher = self.create_publisher(GnssStatus, GNSS_STATUS, 10)
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        if not self.should_simulate:
            from .gnss_sensor import GnssSensor

            sensor = GnssSensor()
            self.get_current_position = sensor.get_current_position
        else:
            from .gnss_sim import GnssSimulator

            simulator = GnssSimulator(self)
            self.get_current_position = simulator.get_current_position

        self.poller = self.create_timer(1.0 / self.update_frequency, self.publish)
        self.get_logger().info(
            "{}GNSS node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def publish(self):
        lat, lon = self.get_current_position()

        if isinstance(lat, float) and isinstance(lon, float):
            msg = GnssStatus()
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
