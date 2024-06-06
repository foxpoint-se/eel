#!/usr/bin/env python3
from .localizer import Localizer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from eel_interfaces.msg import Coordinate, ImuStatus, PressureStatus
from ..utils.topics import (
    GNSS_STATUS,
    MOTOR_CMD,
    IMU_STATUS,
    LOCALIZATION_STATUS,
    PRESSURE_STATUS,
)
from ..utils.sim import LINEAR_VELOCITY


FORWARD_MAX_SPEED = LINEAR_VELOCITY
REVERSE_MAX_SPEED = 0.2 * FORWARD_MAX_SPEED


def calculate_speed_from_motor_mps(motor_speed: float) -> float:
    if motor_speed > 0:
        return motor_speed * FORWARD_MAX_SPEED
    elif motor_speed < 0:
        return motor_speed * REVERSE_MAX_SPEED
    return 0


class Localization(Node):
    def __init__(self):
        super().__init__("localization", parameter_overrides=[])
        self.update_frequency_hz = 5
        self.gnss_subscription = self.create_subscription(
            Coordinate, GNSS_STATUS, self.handle_gnss_msg, 10
        )
        self.motor_subscription = self.create_subscription(
            Float32, MOTOR_CMD, self.handle_motor_msg, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_msg, 10
        )
        self.pressure_subscription = self.create_subscription(
            PressureStatus,
            PRESSURE_STATUS,
            self.handle_pressure_status_msg,
            10,
        )
        self.status_publisher = self.create_publisher(
            Coordinate, LOCALIZATION_STATUS, 10
        )
        self.localizer = Localizer()
        self.loop = self.create_timer(1.0 / self.update_frequency_hz, self.do_work)
        self.get_logger().info("Localization node started")

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

    def do_work(self) -> None:
        calculated_position = self.localizer.get_calculated_position(
            current_time_sec=time.time()
        )
        if calculated_position:
            msg = Coordinate()
            msg.lat = calculated_position["lat"]
            msg.lon = calculated_position["lon"]
            self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
