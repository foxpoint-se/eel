#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Callable, Optional, TypedDict, cast, Sequence
import json

from awscrt import mqtt, io
from awsiot import mqtt_connection_builder
from std_msgs.msg import Float32
from eel_interfaces.msg import ImuStatus

from ..utils.topics import (
    MOTOR_CMD,
    IMU_STATUS,
    RUDDER_X_CMD,
    RUDDER_Y_CMD,
)


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


class FloatMsg(TypedDict):
    data: float


SubscriberCallback = Callable[[str, bytes, bool, mqtt.QoS, bool], None]


def init_one_mqtt_sub(
    mqtt_conn: mqtt.Connection, topic: str, callback: SubscriberCallback
) -> None:
    mqtt_conn.subscribe(
        topic=topic,
        qos=mqtt.QoS.AT_LEAST_ONCE,
        callback=callback,
    )


# usage:
# ros2 run eel mqtt_bridge --ros-args -p path_for_config:=/path/to/config.json
class MqttBridge(Node):
    def __init__(self):
        super().__init__("mqtt_bridge_node", parameter_overrides=[])

        self.mqtt_conn: Optional[mqtt.Connection] = None

        self.declare_parameter("path_for_config", "")

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
        self.init_ros_pubs()
        self.init_mqtt_subs()

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

    def init_mqtt_subs(self) -> None:
        topics_and_callbacks: Sequence[tuple[str, SubscriberCallback]] = [
            (f"{self.robot_name}/{MOTOR_CMD}", self.handle_incoming_motor_cmd),
            (
                f"{self.robot_name}/{RUDDER_X_CMD}",
                self.handle_incoming_rudder_horizontal,
            ),
            (
                f"{self.robot_name}/{RUDDER_Y_CMD}",
                self.handle_incoming_rudder_vertical,
            ),
        ]
        if self.mqtt_conn:
            for t in topics_and_callbacks:
                topic = t[0]
                callback = t[1]
                init_one_mqtt_sub(
                    mqtt_conn=self.mqtt_conn, topic=topic, callback=callback
                )
                self.get_logger().info(f"Subscribed to MQTT topic {topic}")
        else:
            self.get_logger().info("Could not subscribe. Connection is undefined.")

    def handle_incoming_motor_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsg, json.loads(payload))
        motor_value = float(converted["data"])
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

    def handle_incoming_rudder_horizontal(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsg, json.loads(payload))
        value = float(converted["data"])
        msg = Float32()
        msg.data = value
        self.rudder_horizontal_publisher.publish(msg)

    def handle_incoming_rudder_vertical(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsg, json.loads(payload))
        value = float(converted["data"])
        msg = Float32()
        msg.data = value
        self.rudder_vertical_publisher.publish(msg)

    def init_ros_pubs(self) -> None:
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_horizontal_publisher = self.create_publisher(
            Float32, RUDDER_X_CMD, 10
        )
        self.rudder_vertical_publisher = self.create_publisher(
            Float32, RUDDER_Y_CMD, 10
        )

    def init_subs(self) -> None:
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.imu_status_callback, 10
        )

    def imu_status_callback(self, msg: ImuStatus) -> None:
        if self.mqtt_conn:
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
