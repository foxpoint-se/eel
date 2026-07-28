from launch_ros.actions import Node

from launch import LaunchDescription

SIMULATE_PARAM = "simulate"
MOTOR_PIN_PARAM = "motor_pin"
DIRECTION_PIN_PARAM = "direction_pin"
DISTANCE_SENSOR_CHANNEL_PARAM = "distance_sensor_channel"
CMD_TOPIC_PARAM = "cmd_topic"
STATUS_TOPIC_PARAM = "status_topic"
TANK_FLOOR_VALUE_PARAM = "tank_floor_value"
TANK_CEILING_VALUE_PARAM = "tank_ceiling_value"

FRONT_TANK_CMD = "tank_front/cmd"
FRONT_TANK_STATUS = "tank_front/status"
REAR_TANK_CMD = "tank_rear/cmd"
REAR_TANK_STATUS = "tank_rear/status"


def _tank_node(
    name: str,
    cmd_topic: str,
    status_topic: str,
    motor_pin: int,
    direction_pin: int,
    channel: int,
    floor: float,
    ceiling: float,
) -> Node:
    return Node(
        package="eel",
        executable="tank",
        name=name,
        parameters=[
            {SIMULATE_PARAM: True},
            {CMD_TOPIC_PARAM: cmd_topic},
            {STATUS_TOPIC_PARAM: status_topic},
            {MOTOR_PIN_PARAM: motor_pin},
            {DIRECTION_PIN_PARAM: direction_pin},
            {DISTANCE_SENSOR_CHANNEL_PARAM: channel},
            {TANK_FLOOR_VALUE_PARAM: floor},
            {TANK_CEILING_VALUE_PARAM: ceiling},
        ],
    )


def generate_launch_description():
    simulate = {SIMULATE_PARAM: True}

    return LaunchDescription(
        [
            Node(package="eel", executable="imu", name="imu_node", parameters=[simulate]),
            Node(package="eel", executable="motor", name="motor_node", parameters=[simulate]),
            Node(package="eel", executable="rudder", name="rudder_node", parameters=[simulate]),
            Node(package="eel", executable="battery", name="battery_node", parameters=[simulate]),
            Node(package="eel", executable="pressure", name="pressure_node", parameters=[simulate]),
            _tank_node("front_tank", FRONT_TANK_CMD, FRONT_TANK_STATUS, 23, 18, 0, 0.66, 0.16),
            _tank_node("rear_tank", REAR_TANK_CMD, REAR_TANK_STATUS, 24, 25, 1, 0.325, 0.005),
            Node(package="eel", executable="leakage", name="leakage_node", parameters=[simulate]),
            Node(package="eel", executable="modem", name="modem_node", parameters=[simulate]),
            Node(package="eel", executable="localization", name="localization"),
            Node(package="eel", executable="navigate", name="navigation_action_server"),
            Node(package="eel", executable="navigate_client", name="navigation_action_client"),
            Node(package="eel", executable="depth_control_rudder", name="depth_control_rudder_node"),
            Node(package="eel", executable="depth_control", name="depth_control_tank_node"),
            Node(package="eel", executable="dive", name="dive_action_server"),
            Node(package="eel", executable="dive_client", name="dive_action_client"),
            Node(package="eel", executable="data_logger", name="data_logger_node"),
        ]
    )
