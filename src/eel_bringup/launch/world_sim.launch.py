"""Full sim: plant + GUI + device logic + app.

Plant publishes battery/leakage/modem raw stubs.

ros2 launch eel_bringup world_sim.launch.py
ros2 launch eel_bringup world_sim.launch.py depth_mode:=tanks
"""

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def _launch_world_sim(context: LaunchContext, *args, **kwargs):
    depth_mode = LaunchConfiguration("depth_mode").perform(context)
    if depth_mode not in ("rudder", "tanks"):
        raise ValueError(f"depth_mode must be 'rudder' or 'tanks', got {depth_mode!r}")

    actions: list = [
        Node(package="eel", executable="motor", name="motor", output="screen"),
        Node(
            package="eel",
            executable="rudder",
            name="rudder",
            output="screen",
            # GUI republishes cmds; long timeout avoids nuisance centering while clicking.
            parameters=[{"cmd_timeout_s": 5.0}],
        ),
        Node(
            package="eel_world_sim",
            executable="world_sim",
            name="world_sim",
            output="screen",
            remappings=[("world/depth", "pressure/depth_m")],
        ),
        Node(package="eel", executable="pressure", name="pressure", output="screen"),
        Node(package="eel", executable="gnss", name="gnss", output="screen"),
        Node(package="eel", executable="imu", name="imu", output="screen"),
        Node(
            package="eel",
            executable="tank",
            name="front_tank",
            output="screen",
            parameters=[
                {"cmd_topic": "tank_front/cmd"},
                {"status_topic": "tank_front/status"},
            ],
        ),
        Node(
            package="eel",
            executable="tank",
            name="rear_tank",
            output="screen",
            parameters=[
                {"cmd_topic": "tank_rear/cmd"},
                {"status_topic": "tank_rear/status"},
            ],
        ),
        Node(package="eel", executable="battery", name="battery", output="screen"),
        Node(package="eel", executable="leakage", name="leakage", output="screen"),
        Node(package="eel", executable="modem", name="modem", output="screen"),
        Node(package="eel", executable="led", name="led", output="screen"),
        Node(package="eel", executable="mqtt_bridge_log", name="mqtt_bridge_log", output="screen"),
        Node(package="eel", executable="localization", name="localization", output="screen"),
        Node(package="eel", executable="navigate", name="navigate", output="screen"),
        Node(package="eel", executable="navigate_client", name="navigate_client", output="screen"),
        Node(package="eel", executable="dive", name="dive", output="screen"),
        Node(package="eel", executable="data_logger", name="data_logger", output="screen"),
        Node(package="eel_world_sim", executable="world_sim_gui", name="world_sim_gui", output="screen"),
    ]

    if depth_mode == "rudder":
        actions.append(
            Node(
                package="eel",
                executable="depth_control_rudder",
                name="depth_control_rudder",
                output="screen",
            )
        )
    else:
        actions.append(
            Node(
                package="eel",
                executable="depth_control",
                name="depth_control",
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_mode",
                default_value="rudder",
                description="Depth controller: rudder or tanks",
            ),
            OpaqueFunction(function=_launch_world_sim),
        ]
    )
