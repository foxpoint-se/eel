#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from eel_interfaces.msg import Coordinate, ImuStatus, PressureStatus

from ..utils.sim import LINEAR_VELOCITY
from ..utils.topics import (
    GNSS_STATUS,
    IMU_STATUS,
    LOCALIZATION_DRIFT_BEARING,
    LOCALIZATION_DRIFT_SPEED,
    LOCALIZATION_STATUS,
    MOTOR_CMD,
    PRESSURE_STATUS,
)
from .localizer import Localizer

FORWARD_MAX_SPEED = LINEAR_VELOCITY
REVERSE_MAX_SPEED = 0.2 * FORWARD_MAX_SPEED


def calculate_speed_from_motor_mps(motor_speed: float) -> float:
    if motor_speed > 0:
        return motor_speed * FORWARD_MAX_SPEED
    elif motor_speed < 0:
        return motor_speed * REVERSE_MAX_SPEED
    return 0


class Localization(Node):
    def __init__(self) -> None:
        super().__init__("localization", parameter_overrides=[])
        self.update_frequency_hz = 5
        self.gnss_subscription = self.create_subscription(Coordinate, GNSS_STATUS, self.handle_gnss_msg, 10)
        self.motor_subscription = self.create_subscription(Float32, MOTOR_CMD, self.handle_motor_msg, 10)
        self.imu_subscription = self.create_subscription(ImuStatus, IMU_STATUS, self.handle_imu_msg, 10)
        self.pressure_subscription = self.create_subscription(
            PressureStatus,
            PRESSURE_STATUS,
            self.handle_pressure_status_msg,
            10,
        )
        self.drift_speed_subscription = self.create_subscription(
            Float32, LOCALIZATION_DRIFT_SPEED, self.handle_drift_speed_msg, 10
        )
        self.drift_bearing_subscription = self.create_subscription(
            Float32, LOCALIZATION_DRIFT_BEARING, self.handle_drift_bearing_msg, 10
        )
        self.status_publisher = self.create_publisher(Coordinate, LOCALIZATION_STATUS, 10)

        self.localizer = Localizer()
        self.loop = self.create_timer(1.0 / self.update_frequency_hz, self.do_work)
        self.get_logger().info(f"Localization node started. {LINEAR_VELOCITY} m/s")

    def handle_gnss_msg(self, msg: Coordinate) -> None:
        self.current_gnss_status = msg
        self.localizer.update_known_position({"lat": msg.lat, "lon": msg.lon})

    def handle_motor_msg(self, msg: Float32) -> None:
        current_speed_mps = calculate_speed_from_motor_mps(motor_speed=msg.data)
        self.localizer.update_speed_mps(current_speed_mps)

    def handle_imu_msg(self, msg: ImuStatus) -> None:
        self.localizer.update_heading(msg.heading)

    def handle_pressure_status_msg(self, msg: PressureStatus) -> None:
        self.localizer.update_depth(msg.depth)

    def handle_drift_speed_msg(self, msg: Float32) -> None:
        self.localizer.update_drift_speed_mps(msg.data)

    def handle_drift_bearing_msg(self, msg: Float32) -> None:
        self.localizer.update_drift_bearing(msg.data)

    def do_work(self) -> None:
        calculated_position = self.localizer.get_calculated_position(current_time_sec=time.time())
        if calculated_position:
            msg = Coordinate()
            msg.lat = calculated_position["lat"]
            msg.lon = calculated_position["lon"]
            self.status_publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Localization()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
