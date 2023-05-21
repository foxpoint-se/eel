from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"
MOTOR_PIN_PARAM = "motor_pin"
DIRECTION_PIN_PARAM = "direction_pin"
DISTANCE_SENSOR_PIN_PARAM = "distance_sensor_pin"
CMD_TOPIC_PARAM = "cmd_topic"
STATUS_TOPIC_PARAM = "status_topic"

FRONT_TANK_CMD = "tank_front/cmd"
FRONT_TANK_STATUS = "tank_front/status"
REAR_TANK_CMD = "tank_rear/cmd"
REAR_TANK_STATUS = "tank_rear/status"

TANK_FLOOR_VALUE_PARAM = "tank_floor_value"
TANK_CEILING_VALUE_PARAM = "tank_ceiling_value"


def generate_launch_description():
    ld = LaunchDescription()

    front_tank_node = Node(
        package="eel",
        executable="tank",
        name="front_tank",
        parameters=[
            {SIMULATE_PARAM: False},
            {CMD_TOPIC_PARAM: FRONT_TANK_CMD},
            {STATUS_TOPIC_PARAM: FRONT_TANK_STATUS},
            {MOTOR_PIN_PARAM: "23"},
            {DIRECTION_PIN_PARAM: "18"},
            {DISTANCE_SENSOR_PIN_PARAM: "0"},
            {TANK_FLOOR_VALUE_PARAM: "4736"},
            {TANK_CEILING_VALUE_PARAM: "18256"},
        ],
    )

    rear_tank_node = Node(
        package="eel",
        executable="tank",
        name="rear_tank",
        parameters=[
            {SIMULATE_PARAM: False},
            {CMD_TOPIC_PARAM: REAR_TANK_CMD},
            {STATUS_TOPIC_PARAM: REAR_TANK_STATUS},
            {MOTOR_PIN_PARAM: "24"},
            {DIRECTION_PIN_PARAM: "25"},
            {DISTANCE_SENSOR_PIN_PARAM: "1"},
            {TANK_FLOOR_VALUE_PARAM: "5072"},
            {TANK_CEILING_VALUE_PARAM: "16624"},
        ],
    )

    rudder_node = Node(
        package="eel",
        executable="rudder",
        name="rudder_node",
        parameters=[{SIMULATE_PARAM: False}],
    )

    motor_node = Node(
        package="eel",
        executable="motor",
        name="motor_node",
        parameters=[{SIMULATE_PARAM: False}],
    )

    battery_node = Node(
        package="eel",
        executable="battery",
        name="battery_node",
    )

    ld.add_action(front_tank_node)
    ld.add_action(rear_tank_node)
    ld.add_action(rudder_node)
    ld.add_action(motor_node)
    ld.add_action(battery_node)

    return ld
