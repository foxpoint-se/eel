"""ROS node backing the world-sim GUI (publish cmds, cache telemetry)."""

from __future__ import annotations

import math

import dearpygui.dearpygui as dpg
import rclpy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool, Float32

from eel_interfaces.msg import (
    BatteryStatus,
    Coordinate,
    DepthControlCmd,
    DepthControlStatus,
    ImuStatus,
    NavigationStatus,
    PressureStatus,
    TankStatus,
)
from eel_world_sim.gui.constants import (
    CMD_REPUBLISH_HZ,
    MAP_SIZE_M,
    MAP_ZOOM_DEFAULT_M,
    MAP_ZOOM_MAX_M,
    MAP_ZOOM_MIN_M,
    MOTOR_OFF,
    TRAIL_MAX_POINTS,
    TRAIL_MIN_STEP_M,
)
from eel_world_sim.gui.widgets.battery_status import BatteryStatusView
from eel_world_sim.gui.widgets.depth_control import DEPTH_TARGET_MAX_M, DepthControlStatusView
from eel_world_sim.gui.widgets.imu_status import ImuStatusView
from eel_world_sim.gui.widgets.nav_status import NavStatusView
from eel_world_sim.gui.widgets.tank_status import TankStatusView
from eel_world_sim.local_geo import latlon_to_meters


def _empty_tank_status() -> TankStatusView:
    return TankStatusView(
        current_level=0.0,
        target_level=None,
        target_status="no_target",
        is_autocorrecting=False,
    )


def _tank_status_from_msg(msg: TankStatus) -> TankStatusView:
    target_level = float(msg.target_level[0]) if msg.target_level else None
    return TankStatusView(
        current_level=float(msg.current_level),
        target_level=target_level,
        target_status=str(msg.target_status),
        is_autocorrecting=bool(msg.is_autocorrecting),
    )


class WorldSimGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("world_sim_gui")

        self.declare_parameter("motor_cmd_topic", "motor/cmd")
        self.declare_parameter("rudder_x_cmd_topic", "rudder/cmd_x")
        self.declare_parameter("rudder_y_cmd_topic", "rudder/cmd_y")
        self.declare_parameter("rudder_status_topic", "rudder/status")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("gnss_status_topic", "gnss/status")
        self.declare_parameter("pressure_status_topic", "pressure/status")
        self.declare_parameter("imu_status_topic", "imu/status")
        self.declare_parameter("battery_status_topic", "battery/status")
        self.declare_parameter("leakage_status_topic", "leakage/status")
        self.declare_parameter("nav_status_topic", "nav/status")
        self.declare_parameter("tank_front_status_topic", "tank_front/status")
        self.declare_parameter("tank_rear_status_topic", "tank_rear/status")
        self.declare_parameter("depth_control_cmd_topic", "depth_control/cmd")
        self.declare_parameter("depth_control_enabled_cmd_topic", "depth_control/enabled/cmd")
        self.declare_parameter("depth_control_status_topic", "depth_control/status")

        motor_cmd_topic = self._string_param("motor_cmd_topic")
        rudder_x_topic = self._string_param("rudder_x_cmd_topic")
        rudder_y_topic = self._string_param("rudder_y_cmd_topic")
        rudder_status_topic = self._string_param("rudder_status_topic")
        odom_topic = self._string_param("odom_topic")
        gnss_status_topic = self._string_param("gnss_status_topic")
        status_topic = self._string_param("pressure_status_topic")
        imu_topic = self._string_param("imu_status_topic")
        battery_topic = self._string_param("battery_status_topic")
        leakage_topic = self._string_param("leakage_status_topic")
        nav_topic = self._string_param("nav_status_topic")
        tank_front_topic = self._string_param("tank_front_status_topic")
        tank_rear_topic = self._string_param("tank_rear_status_topic")
        depth_cmd_topic = self._string_param("depth_control_cmd_topic")
        depth_enabled_topic = self._string_param("depth_control_enabled_cmd_topic")
        depth_status_topic = self._string_param("depth_control_status_topic")

        self._motor_cmd = MOTOR_OFF
        self._rudder_x_cmd = 0.0
        self._rudder_y_cmd = 0.0
        self._manual_rudder = True
        self._rudder_status_x = 0.0
        self._rudder_status_y = 0.0
        self._depth_m = 0.0
        self._pitch_deg = 0.0
        self._heading_deg = 0.0
        self._roll_deg = 0.0
        self._pitch_velocity = 0.0
        self._imu_calibrated = False
        self._imu_sys = 0
        self._imu_gyro = 0
        self._imu_accel = 0
        self._imu_mag = 0
        self._battery_voltage_v = 0.0
        self._battery_current_a = 0.0
        self._battery_power_w = 0.0
        self._battery_voltage_ratio = 0.0
        self._leakage = False
        self._nav_auto_mode = False
        self._nav_mission_status = 0
        self._nav_meters_to_target = 0.0
        self._nav_mission_total_meters = 0.0
        self._nav_count_goals_left = 0
        self._tank_front = _empty_tank_status()
        self._tank_rear = _empty_tank_status()
        self._depth_control_enabled = False
        self._depth_control_target_m = 0.0
        self._odom_x_m = 0.0
        self._odom_y_m = 0.0
        self._odom_yaw_deg = 0.0
        self._gnss_east_m = 0.0
        self._gnss_north_m = 0.0
        self._gnss_lat = 0.0
        self._gnss_lon = 0.0
        self._has_gnss = False
        self._trail: list[tuple[float, float]] = []
        self._show_live_trail = True
        self._follow_boat = True
        self._map_visible_m = MAP_ZOOM_DEFAULT_M

        self._motor_pub = self.create_publisher(Float32, motor_cmd_topic, 10)
        self._rudder_x_pub = self.create_publisher(Float32, rudder_x_topic, 10)
        self._rudder_y_pub = self.create_publisher(Float32, rudder_y_topic, 10)
        self.create_subscription(PressureStatus, status_topic, self._on_pressure_status, 10)
        self.create_subscription(ImuStatus, imu_topic, self._on_imu_status, 10)
        self.create_subscription(BatteryStatus, battery_topic, self._on_battery_status, 10)
        self.create_subscription(Bool, leakage_topic, self._on_leakage_status, 10)
        self.create_subscription(NavigationStatus, nav_topic, self._on_nav_status, 10)
        self.create_subscription(TankStatus, tank_front_topic, self._on_tank_front_status, 10)
        self.create_subscription(TankStatus, tank_rear_topic, self._on_tank_rear_status, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Coordinate, gnss_status_topic, self._on_gnss_status, 10)
        self.create_subscription(Vector3, rudder_status_topic, self._on_rudder_status, 10)
        self.create_subscription(DepthControlStatus, depth_status_topic, self._on_depth_control_status, 10)
        self._depth_cmd_pub = self.create_publisher(DepthControlCmd, depth_cmd_topic, 10)
        self._depth_enabled_pub = self.create_publisher(Bool, depth_enabled_topic, 10)
        self.create_timer(1.0 / CMD_REPUBLISH_HZ, self._on_cmd_timer)

        self.get_logger().info(
            "GUI topics "
            f"motor={motor_cmd_topic} rudder_x={rudder_x_topic} rudder_y={rudder_y_topic} "
            f"odom={odom_topic} gnss={gnss_status_topic} status={status_topic}"
        )

    def _string_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _on_pressure_status(self, msg: PressureStatus) -> None:
        self._depth_m = float(msg.depth)

    def _on_imu_status(self, msg: ImuStatus) -> None:
        self._pitch_deg = float(msg.pitch)
        self._heading_deg = float(msg.heading)
        self._roll_deg = float(msg.roll)
        self._pitch_velocity = float(msg.pitch_velocity)
        self._imu_calibrated = bool(msg.is_calibrated)
        self._imu_sys = int(msg.sys)
        self._imu_gyro = int(msg.gyro)
        self._imu_accel = int(msg.accel)
        self._imu_mag = int(msg.mag)

    def _on_battery_status(self, msg: BatteryStatus) -> None:
        self._battery_voltage_v = float(msg.voltage_v)
        self._battery_current_a = float(msg.current_a)
        self._battery_power_w = float(msg.power_w)
        self._battery_voltage_ratio = float(msg.voltage_ratio)

    def _on_leakage_status(self, msg: Bool) -> None:
        self._leakage = bool(msg.data)

    def _on_nav_status(self, msg: NavigationStatus) -> None:
        self._nav_auto_mode = bool(msg.auto_mode_enabled)
        self._nav_mission_status = int(msg.mission_status)
        self._nav_meters_to_target = float(msg.meters_to_target)
        self._nav_mission_total_meters = float(msg.mission_total_meters)
        self._nav_count_goals_left = int(msg.count_goals_left)

    def _on_tank_front_status(self, msg: TankStatus) -> None:
        self._tank_front = _tank_status_from_msg(msg)

    def _on_tank_rear_status(self, msg: TankStatus) -> None:
        self._tank_rear = _tank_status_from_msg(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_x_m = float(msg.pose.pose.position.x)
        self._odom_y_m = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self._odom_yaw_deg = math.degrees(math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))

    def _on_gnss_status(self, msg: Coordinate) -> None:
        self._gnss_lat = float(msg.lat)
        self._gnss_lon = float(msg.lon)
        self._gnss_east_m, self._gnss_north_m = latlon_to_meters(self._gnss_lat, self._gnss_lon)
        self._has_gnss = True
        self._append_trail(self._gnss_east_m, self._gnss_north_m)

    def _append_trail(self, east_m: float, north_m: float) -> None:
        if self._trail:
            last_east, last_north = self._trail[-1]
            if math.hypot(east_m - last_east, north_m - last_north) < TRAIL_MIN_STEP_M:
                return
        self._trail.append((east_m, north_m))
        if len(self._trail) > TRAIL_MAX_POINTS:
            self._trail = self._trail[-TRAIL_MAX_POINTS:]

    def _on_rudder_status(self, msg: Vector3) -> None:
        self._rudder_status_x = float(msg.x)
        self._rudder_status_y = float(msg.y)

    def _on_depth_control_status(self, msg: DepthControlStatus) -> None:
        self._depth_control_enabled = bool(msg.is_enabled)
        self._depth_control_target_m = float(msg.depth_target)

    def set_motor(self, value: float) -> None:
        self._motor_cmd = value
        self._publish_float(self._motor_pub, value)

    def set_rudder_x(self, value: float) -> None:
        self._rudder_x_cmd = value
        if self._manual_rudder:
            self._publish_float(self._rudder_x_pub, value)

    def set_rudder_y(self, value: float) -> None:
        self._rudder_y_cmd = value
        if self._manual_rudder:
            self._publish_float(self._rudder_y_pub, value)

    def set_manual_rudder(self, enabled: bool) -> None:
        self._manual_rudder = enabled
        if enabled:
            self._publish_float(self._rudder_x_pub, self._rudder_x_cmd)
            self._publish_float(self._rudder_y_pub, self._rudder_y_cmd)

    def _publish_float(self, publisher: Publisher, value: float) -> None:
        if not rclpy.ok() or not self.context.ok():
            return
        out = Float32()
        out.data = value
        try:
            publisher.publish(out)
        except Exception:
            pass

    def motor_cmd(self) -> float:
        return self._motor_cmd

    def rudder_x_cmd(self) -> float:
        return self._rudder_x_cmd

    def rudder_y_cmd(self) -> float:
        return self._rudder_y_cmd

    def rudder_status(self) -> tuple[float, float]:
        return self._rudder_status_x, self._rudder_status_y

    def depth_m(self) -> float:
        return self._depth_m

    def pitch_deg(self) -> float:
        return self._pitch_deg

    def imu_status(self) -> ImuStatusView:
        return ImuStatusView(
            heading_deg=self._heading_deg,
            roll_deg=self._roll_deg,
            pitch_deg=self._pitch_deg,
            pitch_velocity=self._pitch_velocity,
            is_calibrated=self._imu_calibrated,
            sys=self._imu_sys,
            gyro=self._imu_gyro,
            accel=self._imu_accel,
            mag=self._imu_mag,
        )

    def battery_status(self) -> BatteryStatusView:
        return BatteryStatusView(
            voltage_v=self._battery_voltage_v,
            current_a=self._battery_current_a,
            power_w=self._battery_power_w,
            voltage_ratio=self._battery_voltage_ratio,
        )

    def leakage(self) -> bool:
        return self._leakage

    def nav_status(self) -> NavStatusView:
        return NavStatusView(
            auto_mode_enabled=self._nav_auto_mode,
            mission_status=self._nav_mission_status,
            meters_to_target=self._nav_meters_to_target,
            mission_total_meters=self._nav_mission_total_meters,
            count_goals_left=self._nav_count_goals_left,
        )

    def tank_front_status(self) -> TankStatusView:
        return self._tank_front

    def tank_rear_status(self) -> TankStatusView:
        return self._tank_rear

    def depth_control_status(self) -> DepthControlStatusView:
        return DepthControlStatusView(
            is_enabled=self._depth_control_enabled,
            depth_target_m=self._depth_control_target_m,
        )

    def set_depth_control_enabled(self, enabled: bool) -> None:
        if not rclpy.ok() or not self.context.ok():
            return
        msg = Bool()
        msg.data = enabled
        try:
            self._depth_enabled_pub.publish(msg)
        except Exception:
            pass

    def set_depth_target(self, depth_m: float) -> None:
        if not rclpy.ok() or not self.context.ok():
            return
        depth_m = min(max(depth_m, 0.0), DEPTH_TARGET_MAX_M)
        msg = DepthControlCmd()
        msg.depth_target = depth_m
        msg.pitch_target = 0.0
        msg.depth_pid_type = "P"
        msg.pitch_pid_type = "P"
        try:
            self._depth_cmd_pub.publish(msg)
        except Exception:
            pass

    def odom_pose(self) -> tuple[float, float, float]:
        return self._odom_x_m, self._odom_y_m, self._odom_yaw_deg

    def gnss_map_pose(self) -> tuple[float, float, float, bool]:
        """East/north meters from map origin, yaw from odom, and whether a fix exists."""
        return self._gnss_east_m, self._gnss_north_m, self._odom_yaw_deg, self._has_gnss

    def gnss_latlon(self) -> tuple[float, float] | None:
        if not self._has_gnss:
            return None
        return self._gnss_lat, self._gnss_lon

    def trail(self) -> list[tuple[float, float]]:
        return self._trail

    def show_live_trail(self) -> bool:
        return self._show_live_trail

    def set_show_live_trail(self, enabled: bool) -> None:
        self._show_live_trail = enabled

    def follow_boat(self) -> bool:
        return self._follow_boat

    def set_follow_boat(self, enabled: bool) -> None:
        self._follow_boat = enabled

    def map_visible_m(self) -> float:
        return self._map_visible_m

    def set_map_visible_m(self, visible_m: float) -> None:
        self._map_visible_m = min(max(visible_m, MAP_ZOOM_MIN_M), MAP_ZOOM_MAX_M)

    def zoom_map_in(self) -> None:
        self.set_map_visible_m(self._map_visible_m * 0.7)

    def zoom_map_out(self) -> None:
        self.set_map_visible_m(self._map_visible_m / 0.7)

    def zoom_map_fit(self) -> None:
        self.set_map_visible_m(MAP_SIZE_M)

    def _on_cmd_timer(self) -> None:
        if not dpg.is_dearpygui_running():
            return
        self._publish_float(self._motor_pub, self._motor_cmd)
        if self._manual_rudder:
            self._publish_float(self._rudder_x_pub, self._rudder_x_cmd)
            self._publish_float(self._rudder_y_pub, self._rudder_y_cmd)
