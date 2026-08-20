"""Full boat stack. Logic + hardware, no plant.

ros2 launch eel_bringup boat.launch.py boat:=alen
ros2 launch eel_bringup boat.launch.py boat:=tvalen depth_mode:=tanks
ros2 launch eel_bringup boat.launch.py boat:=alen pigpiod_host:=pigpiod
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# NOTE: workaround to make relative imports work
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from boat_configs import boat_config  # noqa: E402


def _pair(exec_logic: str, exec_hw: str, name: str, hw_params: list | None = None) -> list:
    nodes = [
        Node(package="eel", executable=exec_logic, name=name),
        Node(
            package="eel",
            executable=exec_hw,
            name=f"{name}_hardware",
            parameters=hw_params or [],
        ),
    ]
    return nodes


def _launch_boat(context: LaunchContext, *args, **kwargs):
    boat = LaunchConfiguration("boat").perform(context)
    pigpiod_host = LaunchConfiguration("pigpiod_host").perform(context)
    depth_mode = LaunchConfiguration("depth_mode").perform(context)
    data_logger = LaunchConfiguration("data_logger").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    path_override = LaunchConfiguration("path_for_config").perform(context)

    if boat not in ("alen", "tvalen"):
        raise ValueError(f"boat must be 'alen' or 'tvalen', got {boat!r}")
    if depth_mode not in ("rudder", "tanks"):
        raise ValueError(f"depth_mode must be 'rudder' or 'tanks', got {depth_mode!r}")

    cfg = boat_config(boat)
    mqtt_path = path_override if path_override else cfg["mqtt_config_path"]

    actions: list = []

    actions.extend(_pair("imu", "imu_hardware", "imu"))
    actions.extend(_pair("motor", "motor_hardware", "motor"))
    actions.extend(
        _pair(
            "rudder",
            "rudder_hardware",
            "rudder",
            hw_params=[{"pigpiod_host": pigpiod_host}],
        )
    )
    actions.extend(_pair("battery", "battery_hardware", "battery"))
    actions.extend(
        _pair(
            "pressure",
            "pressure_hardware",
            "pressure",
            hw_params=[{"serial_port": cfg["pressure_port"]}],
        )
    )
    actions.extend(
        _pair(
            "gnss",
            "gnss_hardware",
            "gnss",
            hw_params=[{"serial_port": cfg["gnss_port"]}],
        )
    )
    actions.extend(_pair("modem", "modem_hardware", "modem"))
    actions.extend(_pair("leakage", "leakage_hardware", "leakage"))
    actions.extend(_pair("led", "led_hardware", "led"))

    tank_launch = os.path.join(
        get_package_share_directory("eel_bringup"),
        "launch",
        "tank_hardware.launch.py",
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tank_launch),
            launch_arguments={"tank": "front"}.items(),
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tank_launch),
            launch_arguments={"tank": "rear"}.items(),
        )
    )

    actions.append(Node(package="eel", executable="localization", name="localization"))
    actions.append(Node(package="eel", executable="navigate", name="navigate"))
    actions.append(Node(package="eel", executable="navigate_client", name="navigate_client"))

    if depth_mode == "rudder":
        actions.append(Node(package="eel", executable="depth_control_rudder", name="depth_control_rudder"))
    else:
        actions.append(Node(package="eel", executable="depth_control", name="depth_control"))

    actions.append(
        Node(
            package="eel",
            executable="mqtt_bridge_aws",
            name="mqtt_bridge_aws",
            parameters=[{"path_for_config": mqtt_path}],
        )
    )

    if data_logger:
        actions.append(Node(package="eel", executable="data_logger", name="data_logger"))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("boat", description="Boat profile: alen or tvalen"),
            DeclareLaunchArgument(
                "pigpiod_host",
                default_value="localhost",
                description="pigpiod host (localhost on Pi; pigpiod in Docker)",
            ),
            DeclareLaunchArgument(
                "depth_mode",
                default_value="rudder",
                description="Depth controller: rudder or tanks",
            ),
            DeclareLaunchArgument(
                "data_logger",
                default_value="false",
                description="Start data_logger node",
            ),
            DeclareLaunchArgument(
                "path_for_config",
                default_value="",
                description="Override MQTT iot_config.json path (empty = boat default)",
            ),
            OpaqueFunction(function=_launch_boat),
        ]
    )
