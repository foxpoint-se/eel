"""Pressure logic: raw depth_m + IMU → PressureStatus."""

from time import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import ImuStatus, PressureStatus

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import IMU_STATUS, PRESSURE_DEPTH_M, PRESSURE_STATUS
from .pressure_math import calculate_center_depth, get_depth_velocity, has_depth_sample

PUBLISH_HZ = 5.0


class PressureNode(Node):
    def __init__(self) -> None:
        super().__init__("pressure")

        self._sensor_depth_m: float | None = None
        self._pitch_deg = 0.0
        self._last_center_depth_m: float | None = None
        self._last_center_at: float | None = None

        self.create_subscription(Float32, PRESSURE_DEPTH_M, self._handle_depth_m, 10)
        self.create_subscription(ImuStatus, IMU_STATUS, self._handle_imu, 10)
        self._pub = self.create_publisher(PressureStatus, PRESSURE_STATUS, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_status)

        self.get_logger().info(f"Pressure started (listening on {PRESSURE_DEPTH_M})")

    def _handle_depth_m(self, msg: Float32) -> None:
        self._sensor_depth_m = float(msg.data)

    def _handle_imu(self, msg: ImuStatus) -> None:
        self._pitch_deg = float(msg.pitch)

    def _publish_status(self) -> None:
        sensor_depth_m = self._sensor_depth_m
        if not has_depth_sample(sensor_depth_m):
            return

        center_depth_m = calculate_center_depth(sensor_depth_m, self._pitch_deg)
        now = time()
        depth_velocity = get_depth_velocity(
            center_depth_m,
            self._last_center_depth_m,
            now,
            self._last_center_at,
        )

        out = PressureStatus()
        out.depth = center_depth_m
        out.depth_velocity = depth_velocity
        self._pub.publish(out)

        self._last_center_depth_m = center_depth_m
        self._last_center_at = now


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PressureNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
