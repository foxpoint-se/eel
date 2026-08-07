#!/usr/bin/env python3
import json
from typing import Callable, List, Mapping, Optional, Sequence, Tuple, TypedDict, cast

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

from ..utils.constants import SIMULATE_PARAM
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
from .mqtt_sim import LoggingMqttBackend, MqttBackend
from .types import BoolMsgMqtt, CoordinateMqtt, to_traced_route_mqtt, transform_coordinate_msg


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


class ImuOffsetsMqtt(TypedDict):
    mag: List[int]
    gyr: List[int]
    acc: List[int]


class BatteryStatusMqtt(TypedDict):
    voltage_ratio: float


class NavigationStatusMqtt(TypedDict):
    meters_to_target: float
    auto_mode_enabled: bool
    waypoints_left: List[CoordinateMqtt]
    count_goals_left: int
    mission_total_meters: float


class TankStatusMqtt(TypedDict):
    current_level: float
    target_level: List[float]
    target_status: str
    is_autocorrecting: bool


class PressureStatusMqtt(TypedDict):
    depth: float
    depth_velocity: float


SubscriberCallback = Callable[[str, bytes, bool, object, bool], None]


def transform_battery_msg(msg: BatteryStatus) -> BatteryStatusMqtt:
    return {"voltage_ratio": msg.voltage_ratio}


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


def transform_imu_offsets_msg(msg: ImuOffsets) -> ImuOffsetsMqtt:
    return {
        "mag": [v for v in msg.mag],
        "acc": [v for v in msg.acc],
        "gyr": [v for v in msg.gyr],
    }


def transform_bool_msg(msg: Bool) -> BoolMsgMqtt:
    return {"data": msg.data}


def transform_nav_status(msg: NavigationStatus) -> NavigationStatusMqtt:
    return {
        "auto_mode_enabled": msg.auto_mode_enabled,
        "count_goals_left": msg.count_goals_left,
        "meters_to_target": msg.meters_to_target,
        "mission_total_meters": msg.mission_total_meters,
        "waypoints_left": [transform_coordinate_msg(w) for w in msg.waypoints_left],
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
    def __init__(self) -> None:
        super().__init__("mqtt_bridge_node", parameter_overrides=[])

        self.declare_parameter(SIMULATE_PARAM, False)
        should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)
        self.declare_parameter("path_for_config", "")

        self.is_connected: bool = False

        mqtt_backend: MqttBackend
        if should_simulate:
            mqtt_backend = LoggingMqttBackend(logger=self.get_logger())
        else:
            from .mqtt_aws import AwsIotMqttBackend, CertData

            path_for_config = self.get_parameter("path_for_config").get_parameter_value().string_value
            with open(path_for_config) as f:
                cert_data = cast(CertData, json.load(f))
            mqtt_backend = AwsIotMqttBackend(cert_data)

        self._mqtt = mqtt_backend
        self.robot_name = mqtt_backend.robot_name

        prefix = "SIMULATE " if should_simulate else ""
        self.get_logger().info(f"{prefix}MQTT bridge node starting...")
        self._mqtt.connect()

        self.init_subs()
        self.init_ros_pubs()
        self.init_mqtt_subs()

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
            (
                f"{self.robot_name}/{DEPTH_CONTROL_CMD}",
                self.handle_incoming_depth_control_cmd,
            ),
            (
                f"{self.robot_name}/{NAVIGATION_LOAD_MISSION}",
                self.handle_incoming_mission,
            ),
            (
                f"{self.robot_name}/{NAVIGATION_LOAD_NAMED_MISSION}",
                self.handle_incoming_named_mission,
            ),
            (
                f"{self.robot_name}/{GNSS_STATUS}",
                self.handle_incoming_gnss_status,
            ),
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
        self.imu_subscription = self.create_subscription(ImuStatus, IMU_STATUS, self.imu_status_callback, 10)
        self.imu_offsets_subscription = self.create_subscription(ImuOffsets, IMU_OFFSETS, self.imu_offsets_callback, 10)
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
        self.modem_status_subscription = self.create_subscription(
            ModemStatus, MODEM_STATUS, self.modem_status_callback, 10
        )
        self.traced_routes_updates_subscription = self.create_subscription(
            TracedRoute, ROUTE_TRACING_UPDATES, self.traced_routes_updates_callback, 10
        )

    def handle_incoming_front_tank_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT front tank cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.front_tank_cmd_publisher.publish(msg)

    def handle_incoming_rear_tank_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT rear tank cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.rear_tank_cmd_publisher.publish(msg)

    def handle_incoming_motor_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT motor cmd payload")
            return
        motor_msg = Float32()
        motor_msg.data = value
        self.motor_publisher.publish(motor_msg)

    def handle_incoming_navigation_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_bool_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT navigation cmd payload")
            return
        msg = Bool()
        msg.data = value
        self.nav_cmd_publisher.publish(msg)

    def handle_incoming_depth_control_cmd(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
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
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT rudder x cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.rudder_horizontal_publisher.publish(msg)

    def handle_incoming_rudder_vertical(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_float_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT rudder y cmd payload")
            return
        msg = Float32()
        msg.data = value
        self.rudder_vertical_publisher.publish(msg)

    def handle_incoming_mission(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
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
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
    ) -> None:
        value = parse_string_payload(payload)
        if value is None:
            self.get_logger().warning("Dropping invalid MQTT named mission payload")
            return
        msg = String()
        msg.data = value
        self.named_mission_publisher.publish(msg)

    def handle_incoming_gnss_status(
        self,
        topic: str,
        payload: bytes,
        dup: bool,
        qos: object,
        retain: bool,
        **_kwargs: object,
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
        topic = f"{self.robot_name}/{BATTERY_STATUS}"
        mqtt_message = transform_battery_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def imu_status_callback(self, msg: ImuStatus) -> None:
        topic = f"{self.robot_name}/{IMU_STATUS}"
        mqtt_message = transform_imu_msg(msg)
        self.publish_mqtt(topic, mqtt_message)

    @throttle(seconds=1)
    def imu_offsets_callback(self, msg: ImuOffsets) -> None:
        topic = f"{self.robot_name}/{IMU_OFFSETS}"
        mqtt_message = transform_imu_offsets_msg(msg)
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

    def traced_routes_updates_callback(self, msg: TracedRoute) -> None:
        topic = f"{self.robot_name}/{ROUTE_TRACING_UPDATES}"
        mqtt_message = to_traced_route_mqtt(msg)
        self.publish_mqtt(topic, mqtt_message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MqttBridge()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
