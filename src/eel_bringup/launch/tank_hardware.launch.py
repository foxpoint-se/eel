"""Boat tank: tank (logic) + tank_hardware (GPIO/sensor).

ros2 launch eel_bringup tank_hardware.launch.py tank:=front
ros2 launch eel_bringup tank_hardware.launch.py tank:=rear
"""

import os
import sys

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

# NOTE: workaround to make relative imports work
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from tank_configs import tank_config  # noqa: E402


def _launch_tank(context: LaunchContext, *args, **kwargs):
    tank = LaunchConfiguration("tank").perform(context)
    if tank not in ("front", "rear"):
        raise ValueError(f"tank must be 'front' or 'rear', got {tank!r}")
    cfg = tank_config(tank)

    logic = Node(
        package="eel",
        executable="tank",
        name=f"{tank}_tank",
        parameters=[
            {"cmd_topic": cfg["cmd_topic"]},
            {"status_topic": cfg["status_topic"]},
        ],
    )
    hardware = Node(
        package="eel",
        executable="tank_hardware",
        name=f"{tank}_tank_hardware",
        parameters=[
            {"status_topic": cfg["status_topic"]},
            {"motor_pin": cfg["motor_pin"]},
            {"direction_pin": cfg["direction_pin"]},
            {"distance_sensor_channel": cfg["distance_sensor_channel"]},
            {"tank_floor_value": cfg["tank_floor_value"]},
            {"tank_ceiling_value": cfg["tank_ceiling_value"]},
        ],
    )
    return [logic, hardware]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("tank", description="Which tank: front or rear"),
            OpaqueFunction(function=_launch_tank),
        ]
    )
