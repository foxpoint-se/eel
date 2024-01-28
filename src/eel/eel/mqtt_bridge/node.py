#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Optional, TypedDict, cast
import json

from awscrt import mqtt, io
from awsiot import mqtt_connection_builder
from std_msgs.msg import Float32
from eel_interfaces.msg import ImuStatus

from ..utils.topics import MOTOR_CMD, IMU_STATUS


class CertData(TypedDict):
    endpoint: str
    port: int
    certificatePath: str
    privateKeyPath: str
    rootCAPath: str
    clientID: str


class ImuStatusMqtt(TypedDict):
    is_calibrated: bool
    sys: int
    gyro: int
    accel: int
    mag: int
    heading: float
    roll: float
    pitch: float
    pitch_velocity: float


# usage:
# ros2 run eel mqtt_bridge --ros-args -p path_for_config:=/path/to/config.json
class MqttBridge(Node):
    def __init__(self):
        super().__init__("mqtt_bridge_node")

        self.mqtt_conn: Optional[mqtt.Connection] = None

        self.declare_parameter("path_for_config", "")

        # self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        path_for_config = (
            self.get_parameter("path_for_config").get_parameter_value().string_value
        )

        self.get_logger().info("MQTT bridge node starting...")

        with open(path_for_config) as f:
            cert_data = json.load(f)
            cert_data = cast(CertData, cert_data)
            self.robot_name = cert_data["clientID"]

        self.get_logger().info("Connecting directly to endpoint")
        self.connect_to_endpoint(cert_data)

        self.init_subs()

    def connect_to_endpoint(self, cert_data: CertData) -> None:
        self.get_logger().info(
            f"Connecting directly to endpoint {cert_data['endpoint']}"
        )
        self.mqtt_conn = mqtt_connection_builder.mtls_from_path(
            endpoint=cert_data["endpoint"],
            port=cert_data["port"],
            cert_filepath=cert_data["certificatePath"],
            pri_key_filepath=cert_data["privateKeyPath"],
            ca_filepath=cert_data["rootCAPath"],
            client_id=cert_data["clientID"],
            http_proxy_options=None,
        )
        connected_future = self.mqtt_conn.connect()
        connected_future.result()
        self.get_logger().info("Connected!")

    def init_subs(self) -> None:
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.imu_status_callback, 10
        )

    def imu_status_callback(self, msg: ImuStatus) -> None:
        topic = f"{self.robot_name}/{IMU_STATUS}"
        mqtt_message: ImuStatusMqtt = {
            "accel": msg.accel,
            "gyro": msg.gyro,
            "heading": msg.heading,
            "is_calibrated": msg.is_calibrated,
            "mag": msg.mag,
            "pitch": msg.pitch,
            "pitch_velocity": msg.pitch_velocity,
            "roll": msg.roll,
            "sys": msg.sys,
        }

        json_payload = json.dumps(mqtt_message)
        self.mqtt_conn.publish(
            topic=topic,
            payload=json_payload,
            qos=mqtt.QoS.AT_LEAST_ONCE,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
