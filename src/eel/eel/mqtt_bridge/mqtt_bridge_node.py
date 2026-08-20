"""MQTT bridge: shared logic, transport injected at startup.

Executables:
  mqtt_bridge_log — LoggingMqttBackend (CI / no certs)
  mqtt_bridge_aws — AwsIotMqttBackend (path_for_config required)
"""

from __future__ import annotations

import json
from typing import List, Mapping, Optional, Sequence, Tuple, cast

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from eel_interfaces.msg import (
    BatteryStatus,
    Coordinate,
    DepthControlCmd,
    ImuOffsets,
    ImuStatus,
    ModemStatus,
    NavigationAssignment,
    NavigationMission,
    NavigationStatus,
    PressureStatus,
    TankStatus,
    TracedRoute,
)

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.throttle import throttle
from ..utils.topics import (
    BATTERY_STATUS,
    DEPTH_CONTROL_CMD,
    FRONT_TANK_CMD,
    FRONT_TANK_STATUS,
    GNSS_STATUS,
    IMU_OFFSETS,
    IMU_STATUS,
    LEAKAGE_STATUS,
    LOCALIZATION_STATUS,
    MODEM_STATUS,
    MOTOR_CMD,
    NAVIGATION_CMD,
    NAVIGATION_LOAD_MISSION,
    NAVIGATION_LOAD_NAMED_MISSION,
    NAVIGATION_STATUS,
    PRESSURE_STATUS,
    REAR_TANK_CMD,
    REAR_TANK_STATUS,
    ROUTE_TRACING_UPDATES,
    RUDDER_X_CMD,
    RUDDER_Y_CMD,
)
from .inbound_validation import (
    parse_bool_payload,
    parse_coordinate_payload,
    parse_depth_control_payload,
    parse_float_payload,
    parse_mission_payload,
    parse_string_payload,
)
from .mqtt_log_backend import LoggingMqttBackend, MqttBackend
from .mqtt_transforms import (
    SubscriberCallback,
    transform_battery_msg,
    transform_bool_msg,
    transform_imu_msg,
    transform_imu_offsets_msg,
    transform_nav_status,
    transform_pressure_status_msg,
    transform_tank_status_msg,
)
from .types import to_traced_route_mqtt, transform_coordinate_msg

PATH_FOR_CONFIG_PARAM = "path_for_config"


class MqttBridge(Node):
    def __init__(self, mqtt_backend: MqttBackend, *, label: str) -> None:
        super().__init__("mqtt_bridge", parameter_overrides=[])
        self.is_connected = False
        self._mqtt = mqtt_backend
        self.robot_name = mqtt_backend.robot_name
        self.get_logger().info(f"{label}MQTT bridge starting...")
        self._mqtt.connect()
        self.init_subs()
        self.init_ros_pubs()
        self.init_mqtt_subs()

    def init_mqtt_subs(self) -> None:
        topics_and_callbacks: Sequence[Tuple[str, SubscriberCallback]] = [
            (f"{self.robot_name}/{MOTOR_CMD}", self.handle_incoming_motor_cmd),
            (f"{self.robot_name}/{RUDDER_X_CMD}", self.handle_incoming_rudder_horizontal),
            (f"{self.robot_name}/{RUDDER_Y_CMD}", self.handle_incoming_rudder_vertical),
            (f"{self.robot_name}/{NAVIGATION_CMD}", self.handle_incoming_navigation_cmd),
            (f"{self.robot_name}/{FRONT_TANK_CMD}", self.handle_incoming_front_tank_cmd),
            (f"{self.robot_name}/{REAR_TANK_CMD}", self.handle_incoming_rear_tank_cmd),
            (f"{self.robot_name}/{DEPTH_CONTROL_CMD}", self.handle_incoming_depth_control_cmd),
            (f"{self.robot_name}/{NAVIGATION_LOAD_MISSION}", self.handle_incoming_mission),
            (f"{self.robot_name}/{NAVIGATION_LOAD_NAMED_MISSION}", self.handle_incoming_named_mission),
            (f"{self.robot_name}/{GNSS_STATUS}", self.handle_incoming_gnss_status),
        ]
        for topic, callback in topics_and_callbacks:
            self._mqtt.subscribe(topic, callback)

    def init_ros_pubs(self) -> None:
        self.motor_publisher = self.create_publisher(Float32, MOTOR_CMD, 10)
        self.rudder_horizontal_publisher = self.create_publisher(Float32, RUDDER_X_CMD, 10)
        self.rudder_vertical_publisher = self.create_publisher(Float32, RUDDER_Y_CMD, 10)
        self.nav_cmd_publisher = self.create_publisher(Bool, NAVIGATION_CMD, 10)
        self.front_tank_cmd_publisher = self.create_publisher(Float32, FRONT_TANK_CMD, 10)
        self.rear_tank_cmd_publisher = self.create_publisher(Float32, REAR_TANK_CMD, 10)
        self.depth_pitch_publisher = self.create_publisher(DepthControlCmd, DEPTH_CONTROL_CMD, 10)
        self.mission_publisher = self.create_publisher(NavigationMission, NAVIGATION_LOAD_MISSION, 10)
        self.named_mission_publisher = self.create_publisher(String, NAVIGATION_LOAD_NAMED_MISSION, 10)
        self.gnss_status_publisher = self.create_publisher(Coordinate, GNSS_STATUS, 10)

    def init_subs(self) -> None:
        self.create_subscription(ImuStatus, IMU_STATUS, self.imu_status_callback, 10)
        self.create_subscription(ImuOffsets, IMU_OFFSETS, self.imu_offsets_callback, 10)
        self.create_subscription(BatteryStatus, BATTERY_STATUS, self.battery_status_callback, 10)
        self.create_subscription(Coordinate, LOCALIZATION_STATUS, self.localization_status_callback, 10)
        self.create_subscription(NavigationStatus, NAVIGATION_STATUS, self.nav_status_callback, 10)
        self.create_subscription(Bool, LEAKAGE_STATUS, self.leakage_status_callback, 10)
        self.create_subscription(TankStatus, FRONT_TANK_STATUS, self.front_tank_status_callback, 10)
        self.create_subscription(TankStatus, REAR_TANK_STATUS, self.rear_tank_status_callback, 10)
        self.create_subscription(PressureStatus, PRESSURE_STATUS, self.pressure_status_callback, 10)
        self.create_subscription(ModemStatus, MODEM_STATUS, self.modem_status_callback, 10)
        self.create_subscription(TracedRoute, ROUTE_TRACING_UPDATES, self.traced_routes_updates_callback, 10)

    def handle_incoming_front_tank_cmd(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT front tank cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.front_tank_cmd_publisher.publish(msg)

    def handle_incoming_rear_tank_cmd(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT rear tank cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.rear_tank_cmd_publisher.publish(msg)

    def handle_incoming_motor_cmd(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT motor cmd payload")
            return
        motor_msg = Float32()
        motor_msg.data = value
        self.motor_publisher.publish(motor_msg)

    def handle_incoming_navigation_cmd(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_bool_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT navigation cmd payload")
            return
        msg = Bool()
        msg.data = value
        self.nav_cmd_publisher.publish(msg)

    def handle_incoming_depth_control_cmd(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        converted = parse_depth_control_payload(payload)
        if converted is None:
            self.get_logger().warning("Dropping invalid MQTT depth control cmd payload")
            return
        msg = DepthControlCmd()
        msg.depth_pid_type = converted["depth_pid_type"]
        msg.depth_target = converted["depth_target"]
        msg.pitch_pid_type = converted["pitch_pid_type"]
        msg.pitch_target = converted["pitch_target"]
        self.depth_pitch_publisher.publish(msg)

    def handle_incoming_rudder_horizontal(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT rudder x cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.rudder_horizontal_publisher.publish(msg)

    def handle_incoming_rudder_vertical(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT rudder y cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.rudder_vertical_publisher.publish(msg)

    def handle_incoming_mission(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        converted = parse_mission_payload(payload)
        if converted is None:
            self.get_logger().warning("Dropping invalid MQTT mission payload")
            return
        msg = NavigationMission()
        ros_assignments: List[NavigationAssignment] = []
        for assignment in converted["assignments"]:
            ros_assignment = NavigationAssignment()
            coord = Coordinate()
            coord.lat = assignment["coordinate"]["lat"]
            coord.lon = assignment["coordinate"]["lon"]
            ros_assignment.coordinate = coord
            ros_assignment.sync_after = assignment["sync_after"]
            ros_assignment.target_depth = assignment["target_depth"]
            ros_assignments.append(ros_assignment)
        msg.assignments = ros_assignments
        self.mission_publisher.publish(msg)

    def handle_incoming_named_mission(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        value = parse_string_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT named mission payload")
            return
        msg = String()
        msg.data = value
        self.named_mission_publisher.publish(msg)

    def handle_incoming_gnss_status(
        self, topic: str, payload: bytes, dup: bool, qos: object, retain: bool, **_kwargs: object
    ) -> None:
        converted = parse_coordinate_payload(payload)
        if converted is None:
            self.get_logger().warning("Dropping invalid MQTT gnss status payload")
            return
        msg = Coordinate()
        msg.lat = converted["lat"]
        msg.lon = converted["lon"]
        self.gnss_status_publisher.publish(msg)

    def publish_mqtt(self, topic: str, mqtt_message: Mapping[str, object]) -> None:
        if self.is_connected:
            self._mqtt.publish(topic, mqtt_message)

    def modem_status_callback(self, msg: ModemStatus) -> None:
        self.is_connected = msg.connectivity

    @throttle(seconds=1)
    def battery_status_callback(self, msg: BatteryStatus) -> None:
        self.publish_mqtt(f"{self.robot_name}/{BATTERY_STATUS}", transform_battery_msg(msg))

    @throttle(seconds=1)
    def imu_status_callback(self, msg: ImuStatus) -> None:
        self.publish_mqtt(f"{self.robot_name}/{IMU_STATUS}", transform_imu_msg(msg))

    @throttle(seconds=1)
    def imu_offsets_callback(self, msg: ImuOffsets) -> None:
        self.publish_mqtt(f"{self.robot_name}/{IMU_OFFSETS}", transform_imu_offsets_msg(msg))

    @throttle(seconds=1)
    def localization_status_callback(self, msg: Coordinate) -> None:
        self.publish_mqtt(f"{self.robot_name}/{LOCALIZATION_STATUS}", transform_coordinate_msg(msg))

    @throttle(seconds=1)
    def nav_status_callback(self, msg: NavigationStatus) -> None:
        self.publish_mqtt(f"{self.robot_name}/{NAVIGATION_STATUS}", transform_nav_status(msg))

    @throttle(seconds=1)
    def leakage_status_callback(self, msg: Bool) -> None:
        self.publish_mqtt(f"{self.robot_name}/{LEAKAGE_STATUS}", transform_bool_msg(msg))

    @throttle(seconds=1)
    def front_tank_status_callback(self, msg: TankStatus) -> None:
        self.publish_mqtt(f"{self.robot_name}/{FRONT_TANK_STATUS}", transform_tank_status_msg(msg))

    @throttle(seconds=1)
    def rear_tank_status_callback(self, msg: TankStatus) -> None:
        self.publish_mqtt(f"{self.robot_name}/{REAR_TANK_STATUS}", transform_tank_status_msg(msg))

    @throttle(seconds=1)
    def pressure_status_callback(self, msg: PressureStatus) -> None:
        self.publish_mqtt(f"{self.robot_name}/{PRESSURE_STATUS}", transform_pressure_status_msg(msg))

    def traced_routes_updates_callback(self, msg: TracedRoute) -> None:
        self.publish_mqtt(f"{self.robot_name}/{ROUTE_TRACING_UPDATES}", to_traced_route_mqtt(msg))


def main_log(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    prelim = Node("mqtt_bridge", parameter_overrides=[])
    backend = LoggingMqttBackend(logger=prelim.get_logger())
    prelim.destroy_node()
    node = MqttBridge(backend, label="LOG ")
    spin_node_until_shutdown(node)


def main_aws(args: Optional[list[str]] = None) -> None:
    from .mqtt_aws import AwsIotMqttBackend, CertData

    rclpy.init(args=args)
    prelim = Node("mqtt_bridge", parameter_overrides=[])
    prelim.declare_parameter(PATH_FOR_CONFIG_PARAM, "")
    path_for_config = prelim.get_parameter(PATH_FOR_CONFIG_PARAM).get_parameter_value().string_value
    prelim.destroy_node()
    if not path_for_config:
        raise ValueError(f"{PATH_FOR_CONFIG_PARAM} is required for mqtt_bridge_aws")

    with open(path_for_config) as f:
        cert_data = cast(CertData, json.load(f))
    backend = AwsIotMqttBackend(cert_data)
    node = MqttBridge(backend, label="AWS ")
    spin_node_until_shutdown(node)
