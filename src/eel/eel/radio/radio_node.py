#!/usr/bin/env python3
import rclpy
import json
from rclpy.node import Node
from eel_interfaces.msg import GnssStatus, ImuStatus, NavigationStatus
from std_msgs.msg import String, Float32, Bool

# from ..utils.serial_helpers import SerialReaderWriter
from ..utils.serial_helpers2 import SerialReaderWriter
from ..utils.radio_helpers.eel_side import (
    EelState,
    from_json_to_command,
    from_state_to_json,
)
from ..utils.topics import (
    RUDDER_CMD,
    MOTOR_CMD,
    IMU_STATUS,
    GNSS_STATUS,
    RADIO_IN,
    RADIO_OUT,
    NAVIGATION_STATUS,
    NAVIGATION_CMD,
)
from ..utils.constants import SIMULATE_PARAM
from ..utils.radio_helpers.ros2dict import ros2dict

RADIO_OUT_2 = "radio2/out"

UPDATE_FREQUENCY = 1


class Radio(Node):
    def __init__(self):
        super().__init__("radio_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value

        self.state = EelState()

        self.latest_messages = {}

        self.sender = self.create_timer(1.0 / UPDATE_FREQUENCY, self.send_state)

        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.handle_imu_msg, 10
        )
        self.gnss_subscription = self.create_subscription(
            GnssStatus, GNSS_STATUS, self.handle_gnss_msg, 10
        )
        self.nav_subscription = self.create_subscription(
            NavigationStatus, NAVIGATION_STATUS, self.handle_nav_msg, 10
        )

        self.radio_out_2_publisher = self.create_publisher(String, RADIO_OUT_2, 10)
        self.radio_out_publisher = self.create_publisher(String, RADIO_OUT, 10)
        self.radio_in_publisher = self.create_publisher(String, RADIO_IN, 10)
        self.rudder_publisher = self.create_publisher(Float32, RUDDER_CMD, 10)
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.nav_publisher = self.create_publisher(Bool, NAVIGATION_CMD, 10)

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

    def handle_gnss_msg(self, msg):
        self.state.update_gnss(msg)  # TODO: remove
        json_msg = ros2dict(msg)
        self.latest_messages[GNSS_STATUS] = json_msg

    def handle_imu_msg(self, msg):
        self.state.update_imu(msg)  # TODO: remove
        json_msg = ros2dict(msg)
        self.latest_messages[IMU_STATUS] = json_msg

    def handle_nav_msg(self, msg):
        self.state.update_nav(msg)  # TODO: remove
        json_msg = ros2dict(msg)
        self.latest_messages[NAVIGATION_STATUS] = json_msg

    def send(self, message=None):
        if message:
            self.reader_writer.send(message)
            radio_out_msg = String()
            radio_out_msg.data = message
            self.radio_out_publisher.publish(radio_out_msg)

    def send2(self, message=None):
        if message:
            self.reader_writer.send(message)
            radio_out_msg = String()
            radio_out_msg.data = message
            self.radio_out_2_publisher.publish(radio_out_msg)

    def send_state(self):
        self.send(message=from_state_to_json(self.state))

        for topic, msg in self.latest_messages.items():
            radio_msg_dict = {}
            radio_msg_dict[topic] = msg
            radio_msg = json.dumps(radio_msg_dict)
            self.send2(radio_msg)

    def handle_incoming_message(self, message):
        radio_in_msg = String()
        radio_in_msg.data = message
        self.radio_in_publisher.publish(radio_in_msg)
        cmd = from_json_to_command(message)
        if cmd.r != None:
            self.handle_rudder_cmd(cmd.r)
        if cmd.m != None:
            self.handle_motor_cmd(cmd.m)
        if cmd.a != None:
            self.handle_nav_radio_cmd(cmd.a)

    def handle_nav_radio_cmd(self, auto_mode_enabled):
        nav_msg = Bool()
        nav_msg.data = bool(auto_mode_enabled)
        self.nav_publisher.publish(nav_msg)

    def handle_rudder_cmd(self, rudder_msg_value):
        rudder_value = float(rudder_msg_value)
        topic_msg = Float32()
        topic_msg.data = rudder_value
        self.rudder_publisher.publish(topic_msg)

    def handle_motor_cmd(self, motor_msg_value):
        motor_value = float(motor_msg_value)
        topic_msg = Float32()
        topic_msg.data = motor_value
        self.motor_publisher.publish(topic_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Radio()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
