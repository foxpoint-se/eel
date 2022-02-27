#!/usr/bin/env python3
from numpy import angle
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ImuStatus
from std_msgs.msg import Float32
from .imu_sensor import ImuSensor
from ..utils.translate import translate_from_range_to_range

IMU_STATUS_TOPIC = "/imu_status"
SIMULATE_PARAM = "simulate"
RUDDER_TOPIC = "rudder"


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(ImuStatus, IMU_STATUS_TOPIC, 10)

        if not self.should_simulate:
            self.poller = self.create_timer(1.0, self.publish_imu)
            self.imu = ImuSensor(self.should_simulate)
        else:
            self.poller = self.create_timer(1.0, self.publish_fake_imu)
            self.current_rudder_angle = float(170)
            self.current_heading = float(0)
            self.rudder_subscription = self.create_subscription(
                Float32, RUDDER_TOPIC, self.handle_rudder_msg, 10
            )

        self.get_logger().info(
            "{}IMU node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def publish_fake_imu(self):
        # if self.current_rudder_angle > 0:
        #     self.current_heading = (self.current_heading + 20) % 360
        # elif self.current_rudder_angle < 0:
        #     self.current_heading = (self.current_heading - 20) % 360
        self.current_heading += translate_from_range_to_range(
            self.current_rudder_angle, -90.0, 90.0, -10.0, 10.0
        )

        heading = self.current_heading
        msg = ImuStatus()
        msg.euler_heading = heading
        msg.is_calibrated = True
        msg.sys = 3
        msg.gyro = 3
        msg.accel = 3
        msg.mag = 3

        self.status_publisher.publish(msg)

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

    def handle_rudder_msg(self, msg):
        self.current_rudder_angle = msg.data * 90


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
