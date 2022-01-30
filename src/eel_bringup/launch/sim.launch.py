from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"


def generate_launch_description():
    ld = LaunchDescription()

    fake_eel_node = Node(
        package="eel",
        executable="fake_eel",
        name="fake_eel",
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
    )

    motor_node = Node(
        package="eel",
        executable="motor",
        name="motor_node",
    )

    ld.add_action(fake_eel_node)
    ld.add_action(radio_node)
    ld.add_action(rudder_node)
    ld.add_action(motor_node)

    return ld
