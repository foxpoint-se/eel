#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import ImuStatus, GnssStatus
import math
from .fake_route import data

GNSS_STATUS_TOPIC = "/gnss_status"
IMU_STATUS_TOPIC = "/imu_status"


def to_degrees(angle):
    return angle * (180 / math.pi)


def to_radians(angle):
    return angle * (math.pi / 180)


def get_heading(lat1, lon1, lat2, lon2):
    y = math.sin(to_radians(lon2 - lon1)) * math.cos(to_radians(lat2))
    x = math.cos(to_radians(lat1)) * math.sin(to_radians(lat2)) - math.sin(
        to_radians(lat1)
    ) * math.cos(to_radians(lat2)) * math.cos(to_radians(lon2 - lon1))
    bearing = math.atan2(y, x)
    return to_degrees(bearing)


class FakeEel(Node):
    def __init__(self):
        super().__init__("fake_eel")
        self.location_list = data
        self.current_index = -1
        self.current_position = data[self.current_index]
        self.current_heading = 0.0

        self.move_timer = self.create_timer(3.0, self.move)
        self.imu_publisher = self.create_publisher(ImuStatus, IMU_STATUS_TOPIC, 10)
        self.gnss_publisher = self.create_publisher(GnssStatus, GNSS_STATUS_TOPIC, 10)
        self.imu_publisher_timer = self.create_timer(1.0, self.publish_imu)
        self.gnss_publisher_timer = self.create_timer(1.0, self.publish_gnss)

        self.get_logger().info(
            "Fake eel node started. Publishing to: {}, {}".format(
                GNSS_STATUS_TOPIC, IMU_STATUS_TOPIC
            )
        )

    def move(self):
        next_index = (self.current_index + 1) % len(self.location_list)
        next_pos = self.location_list[next_index]
        next_heading = get_heading(
            self.current_position["lat"],
            self.current_position["lon"],
            next_pos["lat"],
            next_pos["lon"],
        )
        self.current_index = next_index
        self.current_position = next_pos
        self.current_heading = next_heading

    def publish_imu(self):
        msg = ImuStatus()
        msg.is_calibrated = False
        msg.sys = 1
        msg.gyro = 1
        msg.accel = 1
        msg.mag = 1
        msg.euler_heading = self.current_heading

        self.imu_publisher.publish(msg)

    def publish_gnss(self):
        msg = GnssStatus()
        msg.lat = self.current_position["lat"]
        msg.lon = self.current_position["lon"]

        self.gnss_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeEel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
