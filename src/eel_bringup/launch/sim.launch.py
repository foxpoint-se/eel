from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"
MOTOR_PIN_PARAM = "motor_pin"
DIRECTION_PIN_PARAM = "direction_pin"
DISTANCE_SENSOR_ADDRESS_PARAM = "distance_sensor_address"
CMD_TOPIC_PARAM = "cmd_topic"
STATUS_TOPIC_PARAM = "status_topic"


def generate_launch_description():
    ld = LaunchDescription()

    # navigation_node = Node(
    #     package="eel",
    #     executable="navigation",
    #     name="navigation_node",
    #     parameters=[{SIMULATE_PARAM: True}],
    # )

    # radio_node = Node(
    #     package="eel",
    #     executable="radio",
    #     name="radio_node",
    #     parameters=[{SIMULATE_PARAM: True}],
    # )

    # rudder_node = Node(
    #     package="eel",
    #     executable="rudder",
    #     name="rudder_node",
    #     parameters=[{SIMULATE_PARAM: True}],
    # )

    # motor_node = Node(
    #     package="eel",
    #     executable="motor",
    #     name="motor_node",
    #     parameters=[{SIMULATE_PARAM: True}],
    # )

    # imu_node = Node(
    #     package="eel",
    #     executable="imu",
    #     name="imu_node",
    #     parameters=[{SIMULATE_PARAM: True}],
    # )

    # gnss_node = Node(
    #     package="eel",
    #     executable="gnss",
    #     name="gnss_node",
    #     parameters=[{SIMULATE_PARAM: True}],
    # )

    front_tank_cmd = "tank_front/cmd"
    front_tank_status = "tank_front/status"
    rear_tank_cmd = "tank_rear/cmd"
    rear_tank_status = "tank_rear/status"
    front_tank_node = Node(
        package="eel",
        executable="tank",
        name="tank_node",
        parameters=[
            {SIMULATE_PARAM: True},
            {CMD_TOPIC_PARAM: front_tank_cmd},
            {STATUS_TOPIC_PARAM: front_tank_status},
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
            {CMD_TOPIC_PARAM: rear_tank_cmd},
            {STATUS_TOPIC_PARAM: rear_tank_status},
            {MOTOR_PIN_PARAM: "22"},
            {DIRECTION_PIN_PARAM: "23"},
            {DISTANCE_SENSOR_ADDRESS_PARAM: "56"},
        ],
    )

    # ld.add_action(navigation_node)
    # ld.add_action(radio_node)
    # ld.add_action(rudder_node)
    # ld.add_action(motor_node)
    # ld.add_action(imu_node)
    # ld.add_action(gnss_node)
    ld.add_action(front_tank_node)
    ld.add_action(rear_tank_node)

    return ld
