from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"


def generate_launch_description():
    ld = LaunchDescription()

    imu_node = Node(
        package="eel",
        executable="imu",
        name="imu_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    rudder_node = Node(
        package="eel",
        executable="rudder",
        name="rudder_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    motor_node = Node(
        package="eel",
        executable="motor",
        name="motor_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    localization = Node(
        package="eel",
        executable="localization",
        name="localization",
    )

    pressure_node = Node(
        package="eel",
        executable="pressure",
        name="pressure_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    ld.add_action(imu_node)
    ld.add_action(rudder_node)
    ld.add_action(motor_node)
    ld.add_action(localization)
    ld.add_action(pressure_node)

    return ld
