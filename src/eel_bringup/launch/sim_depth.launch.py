from launch import LaunchDescription
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


def generate_launch_description():
    ld = LaunchDescription()

    front_tank_node = Node(
        package="eel",
        executable="tank",
        name="front_tank",
        parameters=[
            {SIMULATE_PARAM: True},
            {CMD_TOPIC_PARAM: FRONT_TANK_CMD},
            {STATUS_TOPIC_PARAM: FRONT_TANK_STATUS},
            {MOTOR_PIN_PARAM: "23"},
            {DIRECTION_PIN_PARAM: "18"},
            {DISTANCE_SENSOR_CHANNEL_PARAM: "0"},
            {TANK_FLOOR_VALUE_PARAM: "0.66"},
            {TANK_CEILING_VALUE_PARAM: "0.16"},
        ],
    )

    rear_tank_node = Node(
        package="eel",
        executable="tank",
        name="rear_tank",
        parameters=[
            {SIMULATE_PARAM: True},
            {CMD_TOPIC_PARAM: REAR_TANK_CMD},
            {STATUS_TOPIC_PARAM: REAR_TANK_STATUS},
            {MOTOR_PIN_PARAM: "24"},
            {DIRECTION_PIN_PARAM: "25"},
            {DISTANCE_SENSOR_CHANNEL_PARAM: "1"},
            {TANK_FLOOR_VALUE_PARAM: "0.288"},
            {TANK_CEILING_VALUE_PARAM: "0.005"},
        ],
    )

    pressure_node = Node(
        package="eel",
        executable="pressure",
        name="pressure_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    imu_node = Node(
        package="eel",
        executable="imu",
        name="imu_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    depth_control_node = Node(
        package="eel",
        executable="depth_control",
        name="depth_control_node",
    )

    ld.add_action(front_tank_node)
    ld.add_action(rear_tank_node)
    ld.add_action(pressure_node)
    ld.add_action(imu_node)
    ld.add_action(depth_control_node)

    return ld
