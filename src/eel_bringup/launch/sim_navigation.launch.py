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
