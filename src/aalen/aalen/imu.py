#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import adafruit_bno055
import board
from aalen_interfaces.msg import ImuStatus

IMU_STATUS_TOPIC = '/imu_status'

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.imu_publisher = self.create_publisher(ImuStatus, IMU_STATUS_TOPIC, 10)
        self.imu_poller = self.create_timer(1.0, self.publish_imu)
        self.get_logger().info("IMU status publisher has been started.")

        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)


    def publish_imu(self):
        msg = ImuStatus()
        heading, roll, pitch = self.sensor.euler
        sys, gyro, accel, mag = self.sensor.calibration_status
        is_calibrated = self.sensor.calibrated

        msg.is_calibrated = is_calibrated or False
        msg.sys = sys or 0
        msg.gyro = gyro or 0
        msg.accel = accel or 0
        msg.mag = mag or 0
        msg.euler_heading = float(heading or 0)

        self.imu_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
