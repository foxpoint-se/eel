#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ImuStatus
from .imu_sensor import ImuSensor
from .imu_sim import ImuSimulator

IMU_STATUS_TOPIC = "/imu_status"
SIMULATE_PARAM = "simulate"


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(ImuStatus, IMU_STATUS_TOPIC, 10)

        if not self.should_simulate:
            sensor = ImuSensor()
            self.get_heading = sensor.get_heading
            self.get_calibration_status = sensor.get_calibration_status
            self.get_is_calibrated = sensor.get_is_calibrated

        else:
            simulator = ImuSimulator(self)
            self.get_heading = simulator.get_heading
            self.get_calibration_status = simulator.get_calibration_status
            self.get_is_calibrated = simulator.get_is_calibrated

        self.poller = self.create_timer(1.0, self.publish_imu)

        self.get_logger().info(
            "{}IMU node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def publish_imu(self):
        heading = self.get_heading()
        sys, gyro, accel, mag = self.get_calibration_status()
        is_calibrated = self.get_is_calibrated()

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
