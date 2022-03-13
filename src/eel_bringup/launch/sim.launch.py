from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"


def generate_launch_description():
    ld = LaunchDescription()

    navigation_node = Node(
        package="eel",
        executable="navigation",
        name="navigation_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    radio_node = Node(
        package="eel",
        executable="radio",
        name="radio_node",
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

    imu_node = Node(
        package="eel",
        executable="imu",
        name="imu_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    gnss_node = Node(
        package="eel",
        executable="gnss",
        name="gnss_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    ld.add_action(navigation_node)
    ld.add_action(radio_node)
    ld.add_action(rudder_node)
    ld.add_action(motor_node)
    ld.add_action(imu_node)
    ld.add_action(gnss_node)

    return ld
