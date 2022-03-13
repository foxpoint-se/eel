#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus, ImuStatus
from std_msgs.msg import String, Float32

GNSS_STATUS_TOPIC = "gnss_status"
IMU_STATUS_TOPIC = "imu_status"
MOTOR_TOPIC = "motor"
RUDDER_TOPIC = "rudder"


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation")
        self.EARTH_RADIUS = 6371000
        self._travel_plan = [{"lat":59.30938741468102,"lon":17.975370883941654}, {"lat":59.30978752690931,"lon":17.97597169876099}, {"lat":59.30937966322808,"lon":17.976068258285526}]
        self.position_index = 0
        self.current_position = {"lat": None, "long": None}
        self.target = self._travel_plan[self.position_index]

        self.distance_to_target = 9999999999.0
        self.bearing_to_target = 0.0

        self.current_heading = 0.0

        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS_TOPIC, self.handle_gnss_update, 10
        )
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS_TOPIC, self.handle_imu_update, 10
        )

        self.motor_publisher = self.create_publisher(Float32, MOTOR_TOPIC, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_TOPIC, 10)

    def handle_imu_update(self, msg):
        self.current_heading = msg.euler_heading

    def handle_gnss_update(self, msg):
        data = {
            "lat": msg.lat,
            "lon": msg.lon,
        }

        self.current_position = data
        self.distance_to_target = self.calc_distance(self.current_position, self.target)
        self.bearing_to_target = self.calc_bearing(self.current_position, self.target)

        degree_diff = abs(self.bearing_to_target - self.current_heading) % 360
        degree_diff = 360 - degree_diff if degree_diff > 180 else degree_diff
        direction = self.find_turn_side(self.current_heading, self.bearing_to_target)

        prop_diff = degree_diff / 180.0
        rudder_angle = 1 if prop_diff > 0.1 else (prop_diff * 3)
        rudder_angle = rudder_angle * direction

        self.get_logger().info("Distance to target is: {}m\tBearing to target is: {}\nHeading: {}, Direction: {}\tDegree diff: {}\tRudder angle: {}".format(
            self.distance_to_target, self.bearing_to_target, self.current_heading, direction, degree_diff, rudder_angle))

        if self.distance_to_target < 5.0:
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

    def find_turn_side(self, curr, target):
        diff = target - curr

        if diff < 0:
            diff += 360

        return -1 if diff > 180 else 1

    def calc_distance(self, pos_1, pos_2):
        """Given two positions as longitude and latitude calculates the distance between the two positions in meters.

        :param pos_1: Position made up of longitude and latitude in signed decimal format
        :param pos_2: Position made up of longitude and latitude in signed decimal format
        :return Distance in meters between positions
        """
        pos_1_lat_rad = pos_1.get("lat") * (math.pi / 180.0)
        pos_2_lat_rad = pos_2.get("lat") * (math.pi / 180.0)
        delta_lat_rad = (pos_1.get("lat") - pos_2.get("lat")) * (math.pi / 180.0)
        delta_long_rad = (pos_1.get("lon") - pos_2.get("lon")) * (math.pi / 180.0)

        a = math.sin(delta_lat_rad / 2.0) * math.sin(delta_lat_rad / 2.0) + \
            math.cos(pos_1_lat_rad) * math.cos(pos_2_lat_rad) * math.sin(delta_long_rad / 2.0) * \
            math.sin(delta_long_rad / 2.0)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = self.EARTH_RADIUS * c

        return distance

    def calc_bearing(self, pos_1, pos_2):
        """Given two positions as longitude and latitude calculates the bearing between the two positions in meters.

        :param pos_1: Position made up of longitude and latitude in signed decimal format
        :param pos_2: Position made up of longitude and latitude in signed decimal format
        :return: Bearing between the two points in true north degree format
        """
        pos_1_lat_rad = pos_1.get("lat") * (math.pi / 180.0)
        pos_2_lat_rad = pos_2.get("lat") * (math.pi / 180.0)
        delta_long_rad = (pos_1.get("lon") - pos_2.get("lon")) * (math.pi / 180.0)

        y = math.sin(delta_long_rad) * math.cos(pos_2_lat_rad)
        x = math.cos(pos_1_lat_rad) * math.sin(pos_2_lat_rad) - math.sin(pos_1_lat_rad) * math.cos(pos_2_lat_rad) * \
            math.cos(delta_long_rad)

        bearing = math.atan2(y, x) * (180.0 / math.pi)
        true_bearing = bearing * -1 if bearing < 0 else 360.0 - bearing

        return true_bearing


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
