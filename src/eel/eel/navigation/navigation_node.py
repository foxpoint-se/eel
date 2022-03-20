#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus, ImuStatus, NavigationStatus, Coordinate
from std_msgs.msg import Float32
from ..utils.nav import (
    get_distance_in_meters,
    get_relative_bearing_in_degrees,
    get_closest_turn_direction,
)
from ..utils.topics import (
    RUDDER_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    GNSS_STATUS,
    NAVIGATION_STATUS,
)

TOLERANCE_IN_METERS = 5.0


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.EARTH_RADIUS = 6371000
        self._travel_plan = [
            {"lat": 59.30938741468102, "lon": 17.975370883941654},
            {"lat": 59.30978752690931, "lon": 17.97597169876099},
            {"lat": 59.30937966322808, "lon": 17.976068258285526},
        ]
        self.position_index = 0
        self.current_position = {"lat": None, "long": None}
        self.target = self._travel_plan[self.position_index]

        self.distance_to_target = 9999999999.0
        self.bearing_to_target = 0.0

        self.current_heading = 0.0

        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS, self.handle_gnss_update, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_update, 10
        )

        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_CMD, 10)
        self.nav_publisher = self.create_publisher(
            NavigationStatus, NAVIGATION_STATUS, 10
        )

    def handle_imu_update(self, msg):
        self.current_heading = msg.euler_heading

    def handle_gnss_update(self, msg):
        self.current_position = {
            "lat": msg.lat,
            "lon": msg.lon,
        }
        self.distance_to_target = get_distance_in_meters(
            self.current_position["lat"],
            self.current_position["lon"],
            self.target["lat"],
            self.target["lon"],
        )
        self.bearing_to_target = get_relative_bearing_in_degrees(
            self.current_position["lat"],
            self.current_position["lon"],
            self.target["lat"],
            self.target["lon"],
        )

        degree_diff = abs(self.bearing_to_target - self.current_heading) % 360
        degree_diff = 360 - degree_diff if degree_diff > 180 else degree_diff
        direction = get_closest_turn_direction(
            self.current_heading, self.bearing_to_target
        )

        prop_diff = degree_diff / 180.0
        rudder_angle = 1 if prop_diff > 0.1 else (prop_diff * 3)
        rudder_angle = rudder_angle * direction

        if self.distance_to_target < TOLERANCE_IN_METERS:
            self.position_index += 1
            if self.position_index < len(self._travel_plan):
                self.target = self._travel_plan[self.position_index]

        motor_value = 1.0 if self.position_index < len(self._travel_plan) else 0.0
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

        rudder_msg = Float32()
        rudder_msg.data = float(rudder_angle)
        self.rudder_publisher.publish(rudder_msg)

        nav_msg = NavigationStatus()
        nav_msg.meters_to_target = self.distance_to_target
        nav_msg.tolerance_in_meters = TOLERANCE_IN_METERS
        nav_msg.next_target = []
        if self.target:
            coordinate = Coordinate()
            coordinate.lat = self.target["lat"]
            coordinate.lon = self.target["lon"]
            nav_msg.next_target.append(coordinate)

        self.nav_publisher.publish(nav_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
