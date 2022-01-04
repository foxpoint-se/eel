#!/usr/bin/env python3
import rclpy
import serial
import pynmea2
from rclpy.node import Node
from aalen_interfaces.msg import GnssStatus

GNSS_STATUS_TOPIC = '/gnss_status'

class GNSS(Node):
    def __init__(self):
        super().__init__("gnss")
        self.publisher = self.create_publisher(GnssStatus, GNSS_STATUS_TOPIC, 10)
        self.gnss_poller = self.create_timer(0.1, self.publish)
        # TODO: remove this comment if we don't seem to have problem with timeout=0
        self.serial_source = serial.Serial("/dev/ttyUSB0", 9600, timeout=0)
        self.get_logger().info("GNSS status publisher has been started.")

    def publish(self):
        line = self.serial_source.readline()
        decoded = line.decode('utf-8')

        # TODO: remove this comment if GGA seems to work instead of GPRMC
        if "GGA" in decoded:
            parsed = pynmea2.parse(decoded)

            # TODO: wrap in debug if statement
            self.get_logger().info("lat: {}, lon: {}".format(parsed.latitude, parsed.longitude))
            
            msg = GnssStatus()
            msg.lat = float(parsed.latitude or 0)
            msg.lon = float(parsed.longitude or 0)

            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
