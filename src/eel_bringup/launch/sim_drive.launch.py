import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"


def generate_launch_description():
    ld = LaunchDescription()

    date_time = f"{datetime.now()}"[:19]
    date_format = date_time.replace(" ", "-").replace(":", "-")
    ros_bag_path = os.path.join(os.getcwd(), "ros_bags", f"{date_format}-recording")

    ros_bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--all", "--storage", "mcap", "--output", ros_bag_path]
    )

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

    modem_node = Node(
        package="eel",
        executable="modem",
        name="modem_node",
        parameters=[{SIMULATE_PARAM: True}]
    )

    battery_node = Node(
        package="eel",
        executable="battery",
        name="battery",
        parameters=[{SIMULATE_PARAM: True}]
    )

    leakage_node = Node(
        package="eel",
        executable="leakage",
        name="leakage",
        parameters=[{SIMULATE_PARAM: True}]
    )

    navigation_server_node = Node(
        package="eel",
        executable="navigate",
        name="navigate"
    )

    navigation_client_node = Node(
        package="eel",
        executable="navigate_client",
        name="navigate_client"
    )

    ld.add_action(ros_bag)
    ld.add_action(imu_node)
    ld.add_action(rudder_node)
    ld.add_action(motor_node)
    ld.add_action(localization)
    ld.add_action(pressure_node)
    ld.add_action(modem_node)
    ld.add_action(battery_node)
    ld.add_action(leakage_node)
    ld.add_action(navigation_server_node)
    ld.add_action(navigation_client_node)

    return ld
