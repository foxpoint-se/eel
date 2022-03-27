#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus, ImuStatus, NavigationStatus, Coordinate
from std_msgs.msg import Float32, Bool
from ..utils.nav import (
    get_distance_in_meters,
    get_relative_bearing_in_degrees,
    get_next_rudder_turn,
)
from ..utils.topics import (
    RUDDER_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    GNSS_STATUS,
    NAVIGATION_STATUS,
    NAVIGATION_CMD,
)

TOLERANCE_IN_METERS = 5.0


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.should_navigate = False
        self._travel_plan = [
            {"lat": 59.30938741468102, "lon": 17.975370883941654},
            {"lat": 59.30978752690931, "lon": 17.97597169876099},
            {"lat": 59.30937966322808, "lon": 17.976068258285526},
        ]
        self.position_index = 0
        self.current_position = {"lat": None, "long": None}
        self.target = self._travel_plan[self.position_index]

        self.distance_to_target = 0.0
        self.bearing_to_target = 0.0

        self.current_heading = 0.0

        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS, self.handle_gnss_update, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_update, 10
        )

        self.nav_cmd_subscriber = self.create_subscription(
            Bool, NAVIGATION_CMD, self.handle_nav_cmd, 10
        )

        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_CMD, 10)
        self.nav_publisher = self.create_publisher(
            NavigationStatus, NAVIGATION_STATUS, 10
        )

    def handle_nav_cmd(self, msg):
        self.should_navigate = msg.data
        self.publish_motor_cmd(0.0)
        self.publish_rudder_cmd(0.0)

    def handle_imu_update(self, msg):
        self.current_heading = msg.euler_heading

    def handle_gnss_update(self, msg):
        self.current_position = {
            "lat": msg.lat,
            "lon": msg.lon,
        }

        if self.target:
            self.distance_to_target = get_distance_in_meters(
                self.current_position["lat"],
                self.current_position["lon"],
                self.target["lat"],
                self.target["lon"],
            )

        if self.should_navigate and self.target:
            self.go_towards_target()

        self.publish_nav_status()

    def publish_nav_status(self):
        nav_msg = NavigationStatus()
        nav_msg.auto_mode_enabled = self.should_navigate
        nav_msg.next_target = []
        if self.target:
            nav_msg.meters_to_target = self.distance_to_target
            nav_msg.tolerance_in_meters = TOLERANCE_IN_METERS
            coordinate = Coordinate()
            coordinate.lat = self.target["lat"]
            coordinate.lon = self.target["lon"]
            nav_msg.next_target.append(coordinate)
        else:
            nav_msg.meters_to_target = 0.0
            nav_msg.tolerance_in_meters = 0.0

        self.nav_publisher.publish(nav_msg)

    def publish_rudder_cmd(self, rudder_turn):
        rudder_msg = Float32()
        rudder_msg.data = float(rudder_turn)
        self.rudder_publisher.publish(rudder_msg)

    def publish_motor_cmd(self, motor_value):
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

    def update_target(self):
        has_next_target = self.position_index + 1 < len(self._travel_plan)
        if has_next_target:
            self.position_index += 1
            self.target = self._travel_plan[self.position_index]
        else:
            self.position_index = None
            self.target = None

    def go_towards_target(self):
        target_reached = self.distance_to_target < TOLERANCE_IN_METERS
        if target_reached:
            self.update_target()

        if self.target:
            motor_value = 1.0
            self.bearing_to_target = get_relative_bearing_in_degrees(
                self.current_position["lat"],
                self.current_position["lon"],
                self.target["lat"],
                self.target["lon"],
            )
            next_rudder_turn = get_next_rudder_turn(
                self.current_heading, self.bearing_to_target
            )
        else:
            motor_value = 0.0
            next_rudder_turn = 0.0

        self.publish_rudder_cmd(next_rudder_turn)
        self.publish_motor_cmd(motor_value)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
