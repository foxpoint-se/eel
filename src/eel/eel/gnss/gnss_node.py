#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus
from .gnss_sensor import GnssSensor
from .gnss_sim import GnssSimulator

GNSS_STATUS_TOPIC = "/gnss_status"
SIMULATE_PARAM = "simulate"
DEBUG_PARAM = "debug"

# hertz (updates per second)
UPDATE_FREQUENCY = 10


class GNSS(Node):
    def __init__(self):
        super().__init__("gnss_node")
        self.publisher = self.create_publisher(GnssStatus, GNSS_STATUS_TOPIC, 10)
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.declare_parameter(DEBUG_PARAM, False)
        self.debug = self.get_parameter(DEBUG_PARAM).value

        if not self.should_simulate:
            sensor = GnssSensor()
            self.get_current_position = sensor.get_current_position
        else:
            simulator = GnssSimulator(self)
            self.get_current_position = simulator.get_current_position

        self.poller = self.create_timer(1.0 / UPDATE_FREQUENCY, self.publish)
        self.get_logger().info(
            "{}GNSS node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def publish(self):
        lat, lon = self.get_current_position()

        if self.debug:
            self.get_logger().info("lat: {}, lon: {}".format(lat, lon))

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
