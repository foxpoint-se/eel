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
        self.serial_source = serial.Serial("/dev/ttyUSB0", 9600, timeout=2)

        # self.hc12_serial = serial.Serial('/dev/ttyS0', 9600)

        self.get_logger().info("GNSS status publisher has been started.")

    # def send(self, message=None):
    #     if message:
    #         self.hc12_serial.write(bytes(message, encoding='utf-8'))

    def publish(self):
        line = self.serial_source.readline()
        decoded = line.decode('utf-8')

        if "GGA" in decoded:
        # if "GPRMC" in decoded:
            parsed = pynmea2.parse(decoded)
            # print(parsed.latitude, parsed.longitude, parsed.timestamp, parsed.timestamp.tzinfo, parsed.timestamp.tzname(), parsed.timestamp.utcoffset())
            
            self.get_logger().info("lat: {}, lon: {}".format(parsed.latitude, parsed.longitude))
            
            # self.send("LA,{}\n".format(parsed.latitude))
            # self.send("LT,{}\n".format(parsed.longitude))

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
