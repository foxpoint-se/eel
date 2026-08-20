"""IMU logic: ImuRaw → ImuStatus (adds pitch velocity)."""

from time import time
from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import ImuRaw, ImuStatus

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import IMU_RAW, IMU_STATUS
from .imu_sensor import get_pitch_velocity


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("imu")
        self._previous_pitch: float | None = None
        self._previous_pitch_at: float | None = None
        self.create_subscription(ImuRaw, IMU_RAW, self._handle_raw, 10)
        self._pub = self.create_publisher(ImuStatus, IMU_STATUS, 10)
        self.get_logger().info(f"IMU started (listening on {IMU_RAW})")

    def _handle_raw(self, msg: ImuRaw) -> None:
        now = time()
        pitch = float(msg.pitch)
        pitch_velocity = get_pitch_velocity(pitch, self._previous_pitch, now, self._previous_pitch_at)

        out = ImuStatus()
        out.is_calibrated = bool(msg.is_calibrated)
        out.sys = int(msg.sys)
        out.gyro = int(msg.gyro)
        out.accel = int(msg.accel)
        out.mag = int(msg.mag)
        out.heading = float(msg.heading)
        out.roll = float(msg.roll)
        out.pitch = pitch
        out.pitch_velocity = pitch_velocity
        self._pub.publish(out)

        self._previous_pitch = pitch
        self._previous_pitch_at = now


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
