#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from aalen_interfaces.msg import GnssStatus, ImuStatus

IMU_STATUS_TOPIC = 'imu_status'
GNSS_STATUS_TOPIC = 'gnss_status'

STATE_KEY = "ST,"
LAT_KEY = "LA,"
LON_KEY = "LT,"
HEADING_KEY = "HE,"
DISTANCE_KEY = "DT,"
OKAY_KEY = "OK,"
SYSTEM_KEY = "SY,"
GYRO_KEY = "GY,"
MAGNETO_KEY = "MA,"
ACCEL_KEY = "AC,"
IS_CALIBRATED_KEY = "IC,"
GNSS_KEY = "GNSS,"
IMU_KEY = "IMU,"

class Radio(Node):
    def __init__(self):
        super().__init__("radio")
        self.hc12_serial = serial.Serial('/dev/ttyS0', 9600)
        self.imu_subscription = self.create_subscription(
                ImuStatus, IMU_STATUS_TOPIC, self.handle_imu_update, 10)
        self.gnss_subscription = self.create_subscription(
                GnssStatus, GNSS_STATUS_TOPIC, self.handle_gnss_update, 10)

        self.get_logger().info("Radio node has been started.")

    def send(self, message=None):
        if message:
            self.hc12_serial.write(bytes("{}\n".format(message), encoding='utf-8'))


    def handle_imu_update(self, msg):
        data = {
            "heading": msg.euler_heading,
            "is_calibrated": msg.is_calibrated,
            "system": msg.sys,
            "gyro": msg.gyro,
            "accelerometer": msg.accel,
            "magnetometer": msg.mag,
        }
        self.send(message="{}{}".format(IMU_KEY, json.dumps(data)))

    def handle_gnss_update(self, msg):
        # self.send(message="{}{}".format(LAT_KEY, msg.lat))
        # self.send(message="{}{}".format(LON_KEY, msg.lon))
        # self.send(message="aaabbbcccdddeeefffggghhhiiijjjkkklllmmmnnnooopppqqqrrrssstttuuuvvvwwwxxxyyyzzzåååäääööö aaabbbcccdddeeefffggghhhiiijjjkkklllmmmnnnooopppqqqrrrssstttuuuvvvwwwxxxyyyzzzåååäääööö aaabbbcccdddeeefffggghhhiiijjjkkklllmmmnnnooopppqqqrrrssstttuuuvvvwwwxxxyyyzzzåååäääööö")
        data = {
            "lat": msg.lat,
            "lon": msg.lon,
        }

        # print(stri)

        self.send(message="{}{}".format(GNSS_KEY, json.dumps(data)))

def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
