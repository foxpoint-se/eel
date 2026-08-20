"""Integration boot test only — co-located with test_stack_publish_status_on_startup.py.

Plant (incl. battery/leakage/modem stubs) + logic (no hardware, no GUI).
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

_INTEGRATION_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="eel", executable="motor", name="motor"),
            Node(package="eel", executable="rudder", name="rudder"),
            Node(
                package="eel_world_sim",
                executable="world_sim",
                name="world_sim",
                remappings=[("world/depth", "pressure/depth_m")],
            ),
            Node(package="eel", executable="pressure", name="pressure"),
            Node(package="eel", executable="gnss", name="gnss"),
            Node(package="eel", executable="imu", name="imu"),
            Node(
                package="eel",
                executable="tank",
                name="front_tank",
                parameters=[{"cmd_topic": "tank_front/cmd", "status_topic": "tank_front/status"}],
            ),
            Node(
                package="eel",
                executable="tank",
                name="rear_tank",
                parameters=[{"cmd_topic": "tank_rear/cmd", "status_topic": "tank_rear/status"}],
            ),
            Node(package="eel", executable="battery", name="battery"),
            Node(package="eel", executable="leakage", name="leakage"),
            Node(package="eel", executable="modem_ci", name="modem"),
            Node(package="eel", executable="led", name="led"),
            Node(package="eel", executable="mqtt_bridge_log", name="mqtt_bridge_log"),
            Node(package="eel", executable="navigate_client", name="navigation_action_client"),
            Node(package="eel", executable="depth_control_rudder", name="depth_control_rudder_node"),
            Node(package="eel", executable="localization", name="localization"),
            Node(package="eel", executable="navigate", name="navigation_action_server"),
            Node(package="eel", executable="dive", name="dive_action_server"),
            ExecuteProcess(
                cmd=[sys.executable, os.path.join(_INTEGRATION_DIR, "dive_demo.py")],
                name="dive_demo",
                output="screen",
            ),
            Node(package="eel", executable="data_logger", name="data_logger_node"),
        ]
    )
