from typing import Literal
from launch_ros.actions import Node


SIMULATE_PARAM = "simulate"

MOTOR_PIN_PARAM = "motor_pin"
DIRECTION_PIN_PARAM = "direction_pin"
DISTANCE_SENSOR_CHANNEL_PARAM = "distance_sensor_channel"
CMD_TOPIC_PARAM = "cmd_topic"
STATUS_TOPIC_PARAM = "status_topic"
FRONT_TANK_CMD = "tank_front/cmd"
FRONT_TANK_STATUS = "tank_front/status"
REAR_TANK_CMD = "tank_rear/cmd"
REAR_TANK_STATUS = "tank_rear/status"

TANK_FLOOR_VALUE_PARAM = "tank_floor_value"
TANK_CEILING_VALUE_PARAM = "tank_ceiling_value"


def create_rudder_node(simulate: bool = False) -> Node:
    return Node(
        package="eel",
        executable="rudder",
        name="rudder_node",
        parameters=[{SIMULATE_PARAM: simulate}],
    )


def create_gnss_node() -> Node:
    return Node(
        package="eel",
        executable="gnss",
        name="gnss_node",
    )


def create_localization_node() -> Node:
    return Node(
        package="eel",
        executable="localization",
        name="localization",
    )


def create_battery_node(simulate: bool = False) -> Node:
    return Node(
        package="eel",
        executable="battery",
        name="battery_node",
        parameters=[{SIMULATE_PARAM: simulate}],
    )


def create_motor_node(simulate: bool = False) -> Node:
    return Node(
        package="eel",
        executable="motor",
        name="motor_node",
        parameters=[{SIMULATE_PARAM: simulate}],
    )


def create_tank_node(
    name: Literal["front_tank", "rear_tank"], simulate: bool = False
) -> Node:
    if name == "front_tank":
        return Node(
            package="eel",
            executable="tank",
            name="front_tank",
            parameters=[
                {SIMULATE_PARAM: simulate},
                {CMD_TOPIC_PARAM: FRONT_TANK_CMD},
                {STATUS_TOPIC_PARAM: FRONT_TANK_STATUS},
                {MOTOR_PIN_PARAM: 23},
                {DIRECTION_PIN_PARAM: 18},
                {DISTANCE_SENSOR_CHANNEL_PARAM: 0},
                {TANK_FLOOR_VALUE_PARAM: 0.66},
                {TANK_CEILING_VALUE_PARAM: 0.16},
            ],
        )
    else:
        return Node(
            package="eel",
            executable="tank",
            name="rear_tank",
            parameters=[
                {SIMULATE_PARAM: simulate},
                {CMD_TOPIC_PARAM: REAR_TANK_CMD},
                {STATUS_TOPIC_PARAM: REAR_TANK_STATUS},
                {MOTOR_PIN_PARAM: 24},
                {DIRECTION_PIN_PARAM: 25},
                {DISTANCE_SENSOR_CHANNEL_PARAM: 1},
                {TANK_FLOOR_VALUE_PARAM: 0.325},
                {TANK_CEILING_VALUE_PARAM: 0.005},
            ],
        )


def create_navigation_node() -> Node:
    return Node(
        package="eel",
        executable="navigate",
        name="navigate",
    )


def create_navigation_client_node() -> Node:
    return Node(
        package="eel",
        executable="navigate_client",
        name="navigate_client",
    )
