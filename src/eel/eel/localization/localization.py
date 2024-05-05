#!/usr/bin/env python3
from typing import TypedDict
from .localizer import LatLon
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from eel_interfaces.msg import GnssStatus, ImuStatus
from ..utils.topics import GNSS_STATUS
from ..utils.sim import LINEAR_VELOCITY
from ..utils.topics import MOTOR_CMD, IMU_STATUS


DEFAULT_START_LAT_LON: LatLon = {"lat": 59.309406850903784, "lon": 17.9742443561554}


class Localization(Node):
    def __init__(self):
        super().__init__("localization", parameter_overrides=[])
        self.update_frequency_hz = 5
        self.declare_parameter("lat", DEFAULT_START_LAT_LON["lat"])
        self.declare_parameter("lon", DEFAULT_START_LAT_LON["lon"])
        lat = self.get_parameter("lat").get_parameter_value().double_value
        lon = self.get_parameter("lon").get_parameter_value().double_value

        if not isinstance(lat, float) or not isinstance(lon, float):
            raise TypeError("lat or lon is either missing or wrong type")

        self.current_position: LatLon = LatLon(lat=lat, lon=lon)

        self.current_gnss_status: GnssStatus = GnssStatus()

        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS, self.handle_gnss_msg, 10
        )

        self.motor_subscription = self.create_subscription(
            Float32, MOTOR_CMD, self.handle_motor_msg, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_msg, 10
        )

        self.loop = self.create_timer(1.0 / self.update_frequency_hz, self.do_work)
        self.get_logger().info("Localization node started")

    def handle_gnss_msg(self, msg: GnssStatus) -> None:
        self.current_gnss_status = msg

    def handle_motor_msg(self, msg: Float32) -> None:
        self.current_motor_msg = msg

    def handle_imu_msg(self, msg: ImuStatus) -> None:
        self.current_imu_msg = msg

    def do_work(self) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
