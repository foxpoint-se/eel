#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus, ImuStatus
from std_msgs.msg import Float32
from geopy import distance
from .gnss_sensor import GnssSensor

GNSS_STATUS_TOPIC = "/gnss_status"
IMU_STATUS_TOPIC = "imu_status"
MOTOR_TOPIC = "motor"
SIMULATE_PARAM = "simulate"
DEBUG_PARAM = "debug"


class GNSS(Node):
    def __init__(self):
        super().__init__("gnss_node")
        self.publisher = self.create_publisher(GnssStatus, GNSS_STATUS_TOPIC, 10)
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.declare_parameter(DEBUG_PARAM, False)
        self.debug = self.get_parameter(DEBUG_PARAM).value

        if not self.should_simulate:
            self.poller = self.create_timer(1.0, self.publish)
            self.gnss = GnssSensor(self.should_simulate)
        else:
            self.motor_subscription = self.create_subscription(
                Float32, MOTOR_TOPIC, self.handle_motor_msg, 10
            )
            self.imu_subscription = self.create_subscription(
                ImuStatus, IMU_STATUS_TOPIC, self.handle_imu_msg, 10
            )
            self.poller = self.create_timer(1.0, self.publish_fake_gnss)
            self.current_position = {"lat": 59.309406850903784, "lon": 17.9742443561554}
            self.speed = 0
            self.current_heading = float(0)

        self.get_logger().info(
            "{}GNSS node started.".format("SIMULATE " if self.should_simulate else "")
        )

    def publish_fake_gnss(self):
        if self.speed > 0:
            new_position = distance.distance(meters=3.0).destination(
                (self.current_position["lat"], self.current_position["lon"]),
                bearing=self.current_heading,
            )
            self.current_position["lat"] = new_position.latitude
            self.current_position["lon"] = new_position.longitude
            # if self.current_heading > 90 and self.current_heading < 270:
            #     self.current_position["lat"] -= 0.0001
            # else:
            #     self.current_position["lat"] += 0.0001

            # if self.current_heading > 0 and self.current_heading < 180:
            #     self.current_position["lon"] += 0.0001
            # else:
            #     self.current_position["lon"] -= 0.0001

        msg = GnssStatus()
        msg.lat = self.current_position.get("lat")
        msg.lon = self.current_position.get("lon")
        self.publisher.publish(msg)

    def handle_motor_msg(self, msg):
        self.speed = msg.data

    def handle_imu_msg(self, msg):
        self.current_heading = msg.euler_heading

    def publish(self):
        lat, lon = self.gnss.get_current_position()

        if self.debug:
            self.get_logger().info("lat: {}, lon: {}".format(lat, lon))

        if isinstance(lat, float) and isinstance(lon, float):
            msg = GnssStatus()
            msg.lat = lat
            msg.lon = lon

            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GNSS()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
