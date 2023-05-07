#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ImuStatus
from time import time
from .imu_sensor import ImuSensor
from .imu_sim import ImuSimulator
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import IMU_STATUS


def get_pitch_velocity(pitch, previous_pitch, now, previous_pitch_at):
    if previous_pitch is None or previous_pitch_at is None:
        return 0.0
    pitch_delta = pitch - previous_pitch
    time_delta = now - previous_pitch_at
    velocity = pitch_delta / time_delta
    return velocity


def filter_pitch(previous, new):
    if previous is None:
        return new
    if abs(new - previous) > 10.0:
        return previous
    if new > 360.0 or new < -360.0:
        return previous
    return new

# example usage: ros2 run eel imu
class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(ImuStatus, IMU_STATUS, 10)
        self.previous_pitch = None
        self.previous_pitch_at = None

        # hertz (publications per second)
        self.update_frequency = 5

        if not self.should_simulate:
            sensor = ImuSensor()
            self.get_euler = sensor.get_euler
            self.get_calibration_status = sensor.get_calibration_status
            self.get_is_calibrated = sensor.get_is_calibrated

        else:
            simulator = ImuSimulator(self)
            self.get_euler = simulator.get_euler
            self.get_calibration_status = simulator.get_calibration_status
            self.get_is_calibrated = simulator.get_is_calibrated

        self.updater = self.create_timer(1.0 / self.update_frequency, self.publish_imu)

        self.get_logger().info(
            "{}IMU node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def publish_imu(self):
        try:
            heading, roll, pitch = self.get_euler()
            sys, gyro, accel, mag = self.get_calibration_status()
            is_calibrated = self.get_is_calibrated()

            msg = ImuStatus()
            msg.is_calibrated = is_calibrated or False
            msg.sys = sys
            msg.gyro = gyro
            msg.accel = accel
            msg.mag = mag
            msg.heading = heading
            msg.roll = roll

            now = time()

            pitch_to_send = filter_pitch(self.previous_pitch, pitch)

            pitch_velocity = get_pitch_velocity(pitch_to_send, self.previous_pitch, now, self.previous_pitch_at)

            msg.pitch = pitch_to_send
            msg.pitch_velocity = pitch_velocity

            self.status_publisher.publish(msg)

            self.previous_pitch = pitch_to_send
            self.previous_pitch_at = now

        except (OSError, IOError) as err:
            self.get_logger().error(str(err))


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
