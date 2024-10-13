#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ImuStatus, ImuOffsets
from time import time
from .imu_sensor import ImuSensor
from .imu_sim import ImuSimulator
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import IMU_STATUS, IMU_OFFSETS


def get_pitch_velocity(pitch, previous_pitch, now, previous_pitch_at):
    if previous_pitch is None or previous_pitch_at is None:
        return 0.0
    pitch_delta = pitch - previous_pitch
    time_delta = now - previous_pitch_at
    velocity = pitch_delta / time_delta
    return velocity

SENSOR_CALIBRATION_OFFSETS = {
    "mag": (193, 80, 84),
    "gyr": (-2, -7, 1),
    "acc": (-5, -13, -12),
}


# example usage: ros2 run eel imu
class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node", parameter_overrides=[])
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(ImuStatus, IMU_STATUS, 10)
        self.offset_publisher = self.create_publisher(ImuOffsets, IMU_OFFSETS, 10)
        self.previous_pitch = None
        self.previous_pitch_at = None

        # hertz (publications per second)
        self.update_frequency = 5
        self.publish_offsets_freq = 0.5
        self.update_calibration_offsets_freq = 0.2

        if not self.should_simulate:
            self.sensor = ImuSensor()

        else:
            self.sensor = ImuSimulator(self)
            
        self.get_euler = self.sensor.get_euler
        self.get_calibration_status = self.sensor.get_calibration_status
        self.get_is_calibrated = self.sensor.get_is_calibrated
        self.get_imu_offsets = self.sensor.get_calibration_offsets

        self.updater = self.create_timer(1.0 / self.update_frequency, self.publish_imu)
        self.imu_offsets_updater = self.create_timer(1.0 / self.publish_offsets_freq, self.publish_imu_offsets)
        self.imu_offsets_writer = self.create_timer(1.0 / self.update_calibration_offsets_freq, self.write_calibration_offsets)

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

            pitch_to_send = pitch

            pitch_velocity = get_pitch_velocity(
                pitch_to_send, self.previous_pitch, now, self.previous_pitch_at
            )

            msg.pitch = pitch_to_send
            msg.pitch_velocity = pitch_velocity

            self.status_publisher.publish(msg)

            self.previous_pitch = pitch_to_send
            self.previous_pitch_at = now

        except (OSError, IOError) as err:
            self.get_logger().error(str(err))

    def publish_imu_offsets(self):
        imu_offsets_map = self.get_imu_offsets()
        
        msg = ImuOffsets()
        msg.mag = list(imu_offsets_map["mag"])
        msg.gyr = list(imu_offsets_map["gyr"])
        msg.acc = list(imu_offsets_map["acc"])

        self.offset_publisher.publish(msg)

    def write_calibration_offsets(self):
        if SENSOR_CALIBRATION_OFFSETS:
            self.sensor.set_offset_values(SENSOR_CALIBRATION_OFFSETS)



def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
