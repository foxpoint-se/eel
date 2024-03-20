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
    RUDDER_X_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    GNSS_STATUS,
    NAVIGATION_STATUS,
    NAVIGATION_CMD,
    NAVIGATION_COORDINATE_UPDATE,
)

TOLERANCE_IN_METERS = 5.0


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.should_navigate = False
        self._coordinates = []
        self.position_index = -1
        self.current_position = {"lat": None, "long": None}
        self.target = None

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

        self.new_coordinate_subscriber = self.create_subscription(
            Coordinate, NAVIGATION_COORDINATE_UPDATE, self.update_coordinates, 10
        )

        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_X_CMD, 10)
        self.nav_publisher = self.create_publisher(NavigationStatus, NAVIGATION_STATUS, 10)

        self.logger = self.get_logger()
        self.logger.info("Navigation node started.")

    def handle_nav_cmd(self, msg):
        self.should_navigate = msg.data

        motor_value = 1.0 if self.should_navigate else 0.0
        self.publish_motor_cmd(motor_value)
        self.publish_rudder_cmd(0.0)

        # If navigation is true, there are coordinates but pos index is -1, we need to initiate
        if self.should_navigate and len(self._coordinates) > 0 and self.position_index == -1:
            self.update_target()

    def handle_imu_update(self, msg):
        self.current_heading = msg.heading

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
            coordinate.index = self.position_index
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
        has_next_target = self.position_index + 1 < len(self._coordinates)
        if has_next_target:
            self.position_index += 1
            self.target = self._coordinates[self.position_index]
        else:
            self.position_index = None
            self.target = None

    def go_towards_target(self):
        target_reached = self.distance_to_target < TOLERANCE_IN_METERS
        if target_reached:
            self.update_target()

        if self.target:
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
            self.publish_motor_cmd(motor_value)

        self.publish_rudder_cmd(next_rudder_turn)
    
    def update_coordinates(self, msg):
        index = msg.index

        # Check if the index value is a positive int, if so verify against current index
        if index >= 0 and self.position_index >= index:
            self.logger.info("Coordinate index {index} not valid since current index at {self.position_index}. Will ignore.")
        
        coordinate_lat = float(msg.lat)
        coordinate_lon = float(msg.lon)

        if index != -1:
            self._coordinates.insert(index, {"lat": coordinate_lat, "lon": coordinate_lon})
        else:
            self._coordinates.append({"lat": coordinate_lat, "lon": coordinate_lon})
        
        self.logger.info(f"Update coordinates with lat {coordinate_lat} and lon {coordinate_lon}")
        self.logger.info(f"Coordinates now: {self._coordinates}")



def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
