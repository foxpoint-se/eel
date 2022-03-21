#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from eel_interfaces.msg import GnssStatus, ImuStatus, NavigationStatus
from std_msgs.msg import String, Float32
from ..utils.serial_helpers import SerialReaderWriter
from ..utils.topics import (
    RUDDER_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    GNSS_STATUS,
    RADIO_IN,
    RADIO_OUT,
    NAVIGATION_STATUS,
)
from ..utils.constants import SIMULATE_PARAM


RUDDER_KEY = "rudder"
MOTOR_KEY = "motor"


class Radio(Node):
    def __init__(self):
        super().__init__("radio_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.state = {
            "heading": 0,
            "isCalibrated": False,
            "system": 0,
            "gyro": 0,
            "accelerometer": 0,
            "magnetometer": 0,
            "lat": 0,
            "lon": 0,
            "nextTarget": None,
            "autoModeEnabled": None,
        }

        self.send_timer = self.create_timer(1.0, self.send_state)

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_update, 10
        )
        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS, self.handle_gnss_update, 10
        )
        self.nav_subscription = self.create_subscription(
            NavigationStatus, NAVIGATION_STATUS, self.handle_nav_update, 10
        )

        self.radio_out_publisher = self.create_publisher(String, RADIO_OUT, 10)
        self.radio_in_publisher = self.create_publisher(String, RADIO_IN, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_CMD, 10)
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)

        serial_port = (
            "/tmp/virtual_serial_eel" if self.should_simulate else "/dev/ttyS0"
        )
        self.reader_writer = SerialReaderWriter(
            serial_port, on_message=self.handle_incoming_message, timeout=None
        )

        self.get_logger().info(
            "{}Radio node started. Port: {}".format(
                "SIMULATE " if self.should_simulate else "", serial_port
            )
        )

    def send(self, message=None):
        if message:
            self.reader_writer.send(message)
            radio_out_msg = String()
            radio_out_msg.data = message
            self.radio_out_publisher.publish(radio_out_msg)

    # TODO: Either we validate the state kind of like this, or perhaps
    # we have a state variable that determines if the state has changed.
    # Or something else. In any case, this prevents the initial zeros
    # from being sent.
    def should_send_state(self):
        return self.state.get("lat") != 0 and self.state.get("lon") != 0
        # return False
        # return True

    def send_state(self):
        if self.should_send_state():
            self.send(message=json.dumps(self.state))

    def update_state(self, data):
        self.state = {**self.state, **data}

    def handle_incoming_message(self, message):
        radio_in_msg = String()
        radio_in_msg.data = message
        self.radio_in_publisher.publish(radio_in_msg)
        data = None
        try:
            data = json.loads(message)
        except:
            self.get_logger().warning(
                "Could not parse json, incoming radio message: {}".format(message)
            )

        if data:
            if data.get(RUDDER_KEY) is not None:
                self.handle_rudder_msg(data.get(RUDDER_KEY))

            if data.get(MOTOR_KEY) is not None:
                self.handle_motor_msg(data.get(MOTOR_KEY))

    def handle_imu_update(self, msg):
        data = {
            "heading": msg.euler_heading,
            "isCalibrated": msg.is_calibrated,
            "system": msg.sys,
            "gyro": msg.gyro,
            "accelerometer": msg.accel,
            "magnetometer": msg.mag,
        }
        self.update_state(data)

    def handle_gnss_update(self, msg):
        data = {
            "lat": msg.lat,
            "lon": msg.lon,
        }

        self.update_state(data)

    def handle_rudder_msg(self, rudder_msg_value):
        rudder_value = None
        try:
            rudder_value = float(rudder_msg_value)
        except:
            self.get_logger().warning(
                "Could not parse float, value {}".format(rudder_msg_value)
            )

        if rudder_value is not None:
            topic_msg = Float32()
            topic_msg.data = rudder_value
            self.rudder_publisher.publish(topic_msg)

    def handle_motor_msg(self, motor_msg_value):
        motor_value = None
        try:
            motor_value = float(motor_msg_value)
        except:
            self.get_logger().warning(
                "Could not parse float, value {}".format(motor_msg_value)
            )

        if motor_value is not None:
            topic_msg = Float32()
            topic_msg.data = motor_value
            self.motor_publisher.publish(topic_msg)

    def handle_nav_update(self, nav_msg):
        next_target = None
        if len(nav_msg.next_target) > 0 and nav_msg.auto_mode_enabled:
            coordinate = nav_msg.next_target[0]
            next_target = {
                "coordinate": {"lat": coordinate.lat, "lon": coordinate.lon},
                "distance": nav_msg.meters_to_target,
                "tolerance": nav_msg.tolerance_in_meters,
            }
        data = {"nextTarget": next_target, "autoModeEnabled": nav_msg.auto_mode_enabled}
        self.update_state(data)


def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
