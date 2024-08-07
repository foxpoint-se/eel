#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Callable, List, Mapping, Optional, Tuple, TypedDict, cast, Sequence
import json

from awscrt import mqtt, io
from awsiot import mqtt_connection_builder
from std_msgs.msg import Float32, Bool
from eel_interfaces.msg import (
    ImuStatus,
    BatteryStatus,
    Coordinate,
    NavigationStatus,
    TankStatus,
    PressureStatus,
)

from ..utils.topics import (
    MOTOR_CMD,
    IMU_STATUS,
    RUDDER_X_CMD,
    RUDDER_Y_CMD,
    BATTERY_STATUS,
    LOCALIZATION_STATUS,
    NAVIGATION_STATUS,
    NAVIGATION_CMD,
    FRONT_TANK_CMD,
    REAR_TANK_CMD,
    LEAKAGE_STATUS,
    REAR_TANK_STATUS,
    FRONT_TANK_STATUS,
    PRESSURE_STATUS,
)
from ..utils.throttle import throttle


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


class BatteryStatusMqtt(TypedDict):
    voltage_percent: float


class FloatMsgMqtt(TypedDict):
    data: float


class BoolMsgMqtt(TypedDict):
    data: bool


class CoordinateMqtt(TypedDict):
    lat: float
    lon: float


class NavigationStatusMqtt(TypedDict):
    meters_to_target: float
    tolerance_in_meters: float
    next_target: List[CoordinateMqtt]
    auto_mode_enabled: bool


class TankStatusMqtt(TypedDict):
    current_level: float
    target_level: List[float]
    target_status: str
    is_autocorrecting: bool


class PressureStatusMqtt(TypedDict):
    depth: float
    depth_velocity: float


SubscriberCallback = Callable[[str, bytes, bool, mqtt.QoS, bool], None]


def init_one_mqtt_sub(
    mqtt_conn: mqtt.Connection, topic: str, callback: SubscriberCallback
) -> None:
    mqtt_conn.subscribe(
        topic=topic,
        qos=mqtt.QoS.AT_LEAST_ONCE,
        callback=callback,
    )


def transform_battery_msg(msg: BatteryStatus) -> BatteryStatusMqtt:
    return {"voltage_percent": msg.voltage_percent}


def transform_imu_msg(msg: ImuStatus) -> ImuStatusMqtt:
    return {
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


def transform_coordinate_msg(msg: Coordinate) -> CoordinateMqtt:
    return {
        "lat": msg.lat,
        "lon": msg.lon,
    }


def transform_bool_msg(msg: Bool) -> BoolMsgMqtt:
    return {"data": msg.data}


def transform_nav_status(msg: NavigationStatus) -> NavigationStatusMqtt:
    return {
        "auto_mode_enabled": msg.auto_mode_enabled,
        "meters_to_target": msg.meters_to_target,
        "tolerance_in_meters": msg.tolerance_in_meters,
        "next_target": [transform_coordinate_msg(t) for t in msg.next_target],
    }


def transform_tank_status_msg(msg: TankStatus) -> TankStatusMqtt:
    return {
        "current_level": msg.current_level,
        "is_autocorrecting": msg.is_autocorrecting,
        "target_level": [float(t) for t in msg.target_level],
        "target_status": msg.target_status,
    }


def transform_pressure_status_msg(msg: PressureStatus) -> PressureStatusMqtt:
    return {"depth": msg.depth, "depth_velocity": msg.depth_velocity}


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
        topics_and_callbacks: Sequence[Tuple[str, SubscriberCallback]] = [
            (f"{self.robot_name}/{MOTOR_CMD}", self.handle_incoming_motor_cmd),
            (
                f"{self.robot_name}/{RUDDER_X_CMD}",
                self.handle_incoming_rudder_horizontal,
            ),
            (
                f"{self.robot_name}/{RUDDER_Y_CMD}",
                self.handle_incoming_rudder_vertical,
            ),
            (
                f"{self.robot_name}/{NAVIGATION_CMD}",
                self.handle_incoming_navigation_cmd,
            ),
            (
                f"{self.robot_name}/{FRONT_TANK_CMD}",
                self.handle_incoming_front_tank_cmd,
            ),
            (
                f"{self.robot_name}/{REAR_TANK_CMD}",
                self.handle_incoming_rear_tank_cmd,
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

    def init_ros_pubs(self) -> None:
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_horizontal_publisher = self.create_publisher(
            Float32, RUDDER_X_CMD, 10
        )
        self.rudder_vertical_publisher = self.create_publisher(
            Float32, RUDDER_Y_CMD, 10
        )
        self.nav_cmd_publisher = self.create_publisher(Bool, NAVIGATION_CMD, 10)
        self.front_tank_cmd_publisher = self.create_publisher(
            Float32, FRONT_TANK_CMD, 10
        )
        self.rear_tank_cmd_publisher = self.create_publisher(Float32, REAR_TANK_CMD, 10)

    def init_subs(self) -> None:
        self.imu_subscription = self.create_subscription(
            ImuStatus, IMU_STATUS, self.imu_status_callback, 10
        )
        self.battery_subscription = self.create_subscription(
            BatteryStatus, BATTERY_STATUS, self.battery_status_callback, 10
        )
        self.localization_subscription = self.create_subscription(
            Coordinate, LOCALIZATION_STATUS, self.localization_status_callback, 10
        )
        self.nav_status_subscription = self.create_subscription(
            NavigationStatus, NAVIGATION_STATUS, self.nav_status_callback, 10
        )
        self.leakage_status_subscription = self.create_subscription(
            Bool, LEAKAGE_STATUS, self.leakage_status_callback, 10
        )
        self.front_tank_subscription = self.create_subscription(
            TankStatus, FRONT_TANK_STATUS, self.front_tank_status_callback, 10
        )
        self.rear_tank_subscription = self.create_subscription(
            TankStatus, REAR_TANK_STATUS, self.rear_tank_status_callback, 10
        )
        self.pressure_status_subscription = self.create_subscription(
            PressureStatus, PRESSURE_STATUS, self.pressure_status_callback, 10
        )

    def handle_incoming_front_tank_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsgMqtt, json.loads(payload))
        msg = Float32()
        msg.data = converted["data"]
        self.front_tank_cmd_publisher.publish(msg)

    def handle_incoming_rear_tank_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsgMqtt, json.loads(payload))
        msg = Float32()
        msg.data = converted["data"]
        self.rear_tank_cmd_publisher.publish(msg)

    def handle_incoming_motor_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsgMqtt, json.loads(payload))
        motor_value = float(converted["data"])
        motor_msg = Float32()
        motor_msg.data = motor_value
        self.motor_publisher.publish(motor_msg)

    def handle_incoming_navigation_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(BoolMsgMqtt, json.loads(payload))
        msg = Bool()
        msg.data = bool(converted["data"])
        self.nav_cmd_publisher.publish(msg)

    def handle_incoming_rudder_horizontal(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: mqtt.QoS,
        retain: bool,
        **kwargs,
    ) -> None:
        converted = cast(FloatMsgMqtt, json.loads(payload))
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
        converted = cast(FloatMsgMqtt, json.loads(payload))
        value = float(converted["data"])
        msg = Float32()
        msg.data = value
        self.rudder_vertical_publisher.publish(msg)

    def publish_mqtt(self, topic: str, mqtt_message: Mapping) -> None:
        if self.mqtt_conn:
            json_payload = json.dumps(mqtt_message)
            self.mqtt_conn.publish(
                topic=topic, payload=json_payload, qos=mqtt.QoS.AT_LEAST_ONCE
            )

    @throttle(seconds=1)
    def battery_status_callback(self, msg: BatteryStatus) -> None:
        topic = f"{self.robot_name}/{BATTERY_STATUS}"
        mqtt_message = transform_battery_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def imu_status_callback(self, msg: ImuStatus) -> None:
        topic = f"{self.robot_name}/{IMU_STATUS}"
        mqtt_message = transform_imu_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def localization_status_callback(self, msg: Coordinate) -> None:
        topic = f"{self.robot_name}/{LOCALIZATION_STATUS}"
        mqtt_message = transform_coordinate_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def nav_status_callback(self, msg: NavigationStatus) -> None:
        topic = f"{self.robot_name}/{NAVIGATION_STATUS}"
        mqtt_message = transform_nav_status(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def leakage_status_callback(self, msg: Bool) -> None:
        topic = f"{self.robot_name}/{LEAKAGE_STATUS}"
        mqtt_message = transform_bool_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def front_tank_status_callback(self, msg: TankStatus) -> None:
        topic = f"{self.robot_name}/{FRONT_TANK_STATUS}"
        mqtt_message = transform_tank_status_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def rear_tank_status_callback(self, msg: TankStatus) -> None:
        topic = f"{self.robot_name}/{REAR_TANK_STATUS}"
        mqtt_message = transform_tank_status_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def pressure_status_callback(self, msg: PressureStatus) -> None:
        topic = f"{self.robot_name}/{PRESSURE_STATUS}"
        mqtt_message = transform_pressure_status_msg(msg)
        self.publish_mqtt(topic, mqtt_message)


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
