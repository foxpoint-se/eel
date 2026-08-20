"""Reads the IMU chip and publishes euler + cal as ImuRaw."""

from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import ImuOffsets, ImuRaw

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import IMU_OFFSETS, IMU_RAW
from .imu_sensor import ImuSensor

PUBLISH_HZ = 5.0
PUBLISH_OFFSETS_HZ = 0.5


class ImuHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_hardware")
        self._sensor = ImuSensor()
        self._raw_pub = self.create_publisher(ImuRaw, IMU_RAW, 10)
        self._offset_pub = self.create_publisher(ImuOffsets, IMU_OFFSETS, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_raw)
        self.create_timer(1.0 / PUBLISH_OFFSETS_HZ, self._publish_offsets)
        self.get_logger().info("IMU hardware started (publishing imu/raw)")

    def _publish_raw(self) -> None:
        try:
            euler = self._sensor.get_euler()
            if euler is None:
                return
            heading, roll, pitch = euler
            sys, gyro, accel, mag = self._sensor.get_calibration_status()
            out = ImuRaw()
            out.is_calibrated = bool(self._sensor.get_is_calibrated())
            out.sys = sys
            out.gyro = gyro
            out.accel = accel
            out.mag = mag
            out.heading = heading
            out.roll = roll
            out.pitch = pitch
            self._raw_pub.publish(out)
        except (OSError, IOError) as err:
            self.get_logger().error(str(err))

    def _publish_offsets(self) -> None:
        try:
            offsets = self._sensor.get_calibration_offsets()
            msg = ImuOffsets()
            msg.mag = list(offsets["mag"])
            msg.gyr = list(offsets["gyr"])
            msg.acc = list(offsets["acc"])
            self._offset_pub.publish(msg)
        except (OSError, IOError) as err:
            self.get_logger().error(str(err))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuHardwareNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
