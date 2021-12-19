#!/usr/bin/env python3
import rclpy
import serial
import pynmea2
from rclpy.node import Node
from example_interfaces.msg import String


class GNSS(Node):
    def __init__(self):
        super().__init__("gnss")
        self.publisher = self.create_publisher(String, "gnss_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish)
        self.serial_source = serial.Serial("/dev/ttyUSB0", 9600, timeout=2)
        self.get_logger().info("GNSS status publisher has been started.")

    def publish(self):
        line = self.serial_source.readline()
        decoded = line.decode('utf-8')

        # if "GGA" in decoded:
        if "GPRMC" in decoded:
            parsed = pynmea2.parse(decoded)
            print(parsed.latitude, parsed.longitude, parsed.timestamp, parsed.timestamp.tzinfo, parsed.timestamp.tzname(), parsed.timestamp.utcoffset())


def main(args=None):
    rclpy.init(args=args)
    node = GNSS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
