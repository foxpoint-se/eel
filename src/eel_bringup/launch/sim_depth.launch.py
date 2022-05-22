from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"
MOTOR_PIN_PARAM = "motor_pin"
DIRECTION_PIN_PARAM = "direction_pin"
DISTANCE_SENSOR_ADDRESS_PARAM = "distance_sensor_address"
CMD_TOPIC_PARAM = "cmd_topic"
STATUS_TOPIC_PARAM = "status_topic"

FRONT_TANK_CMD = "tank_front/cmd"
FRONT_TANK_STATUS = "tank_front/status"
REAR_TANK_CMD = "tank_rear/cmd"
REAR_TANK_STATUS = "tank_rear/status"


def generate_launch_description():
    ld = LaunchDescription()

    front_tank_node = Node(
        package="eel",
        executable="tank",
        name="tank_node",
        parameters=[
            {SIMULATE_PARAM: True},
            {CMD_TOPIC_PARAM: FRONT_TANK_CMD},
            {STATUS_TOPIC_PARAM: FRONT_TANK_STATUS},
            {MOTOR_PIN_PARAM: "22"},
            {DIRECTION_PIN_PARAM: "23"},
            {DISTANCE_SENSOR_ADDRESS_PARAM: "56"},
        ],
    )

    rear_tank_node = Node(
        package="eel",
        executable="tank",
        name="tank_node",
        parameters=[
            {SIMULATE_PARAM: True},
            {CMD_TOPIC_PARAM: REAR_TANK_CMD},
            {STATUS_TOPIC_PARAM: REAR_TANK_STATUS},
            {MOTOR_PIN_PARAM: "22"},
            {DIRECTION_PIN_PARAM: "23"},
            {DISTANCE_SENSOR_ADDRESS_PARAM: "56"},
        ],
    )

    pressure_node = Node(
        package="eel",
        executable="pressure",
        name="pressure_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    ld.add_action(front_tank_node)
    ld.add_action(rear_tank_node)
    ld.add_action(pressure_node)

    return ld
