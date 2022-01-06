#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aalen_interfaces.msg import ImuStatus
from .imu_sensor import ImuSensor

IMU_STATUS_TOPIC = "/imu_status"
SIMULATE_PARAM = "simulate"


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.status_publisher = self.create_publisher(ImuStatus, IMU_STATUS_TOPIC, 10)
        self.poller = self.create_timer(1.0, self.publish_imu)
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.imu = ImuSensor(self.should_simulate)
        self.get_logger().info(
            "IMU status publisher has been started." + " SIMULATE"
            if self.should_simulate
            else ""
        )

    def publish_imu(self):
        heading = self.imu.get_heading()
        sys, gyro, accel, mag = self.imu.get_calibration_status()
        is_calibrated = self.imu.get_is_calibrated()

        msg = ImuStatus()
        msg.is_calibrated = is_calibrated or False
        msg.sys = sys
        msg.gyro = gyro
        msg.accel = accel
        msg.mag = mag
        msg.euler_heading = heading

        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
