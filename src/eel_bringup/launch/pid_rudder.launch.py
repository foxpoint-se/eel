from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    depth_pid_control_node = Node(
        package="eel",
        executable="depth_control_rudder",
        name="depth_control_node_rudder"
    )

    rudder_control_node = Node(
        package="eel",
        executable="rudder",
        name="rudder_node",
    )

    pressure_control_node = Node(
        package="eel",
        executable="pressure",
        name="pressure_control_node",
    )

    imu_control_node = Node(
        package="eel",
        executable="imu",
        name="imu_node"
    )


    ld.add_action(pressure_control_node)
    ld.add_action(rudder_control_node)
    ld.add_action(depth_pid_control_node)
    ld.add_action(imu_control_node)

    return ld
