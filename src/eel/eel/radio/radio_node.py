#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from eel_interfaces.msg import GnssStatus, ImuStatus
from ..utils.serial_helpers import SerialReaderWriter

IMU_STATUS_TOPIC = "imu_status"
GNSS_STATUS_TOPIC = "gnss_status"
SIMULATE_PARAM = "simulate"


class Radio(Node):
    def __init__(self):
        super().__init__("radio_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.state = {
            "heading": 0,
            "isCalibrated": False,
            "system": 0,
            "gyro": 0,
            "accelerometer": 0,
            "magnetometer": 0,
            "lat": 0,
            "lon": 0,
        }

        self.send_timer = self.create_timer(1.0, self.send_state)

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS_TOPIC, self.handle_imu_update, 10
        )
        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS_TOPIC, self.handle_gnss_update, 10
        )

        serial_port = (
            "/tmp/virtual_serial_eel" if self.should_simulate else "/dev/ttyS0"
        )
        self.reader_writer = SerialReaderWriter(
            serial_port, on_message=self.handle_incoming_message
        )

        self.get_logger().info(
            "{}Radio node started. Port: {}".format(
                "SIMULATE " if self.should_simulate else "", serial_port
            )
        )

    def send(self, message=None):
        if message:
            self.reader_writer.send(message)

    def send_state(self):
        self.send(message=json.dumps(self.state))

    def update_state(self, data):
        self.state = {**self.state, **data}

    def handle_incoming_message(self, message):
        self.get_logger().info("INCOMING: {}".format(message))

    def handle_imu_update(self, msg):
        data = {
            "heading": msg.euler_heading,
            "isCalibrated": msg.is_calibrated,
            "system": msg.sys,
            "gyro": msg.gyro,
            "accelerometer": msg.accel,
            "magnetometer": msg.mag,
        }
        self.update_state(data)

    def handle_gnss_update(self, msg):
        data = {
            "lat": msg.lat,
            "lon": msg.lon,
        }

        self.update_state(data)


def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
