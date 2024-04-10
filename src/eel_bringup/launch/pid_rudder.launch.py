from launch import LaunchDescription
from launch_ros.actions import Node

SIMULATE_PARAM = "simulate"


def generate_launch_description():
    ld = LaunchDescription()

    # depth_pid_control_node = Node(
    #     package="eel",
    #     executable="depth_control_rudder",
    #     name="depth_control_node_rudder",
    # )

    # depth_control_node = Node(
    #     package="eel",
    #     executable="depth_control",
    #     name="depth_control_node",
    # )

    rudder_node = Node(
        package="eel",
        executable="rudder",
        name="rudder_node",
        parameters=[{SIMULATE_PARAM: True}],
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

    motor_node = Node(
        package="eel",
        executable="motor",
        name="motor_node",
        parameters=[{SIMULATE_PARAM: True}],
    )

    front_tank_node = Node(
        package="eel",
        executable="tank",
        name="front_tank",
        parameters=[
            {SIMULATE_PARAM: True},
            {"cmd_topic": "tank_front/cmd"},
            {"status_topic": "tank_front/status"},
            {"motor_pin": "23"},
            {"direction_pin": "18"},
            {"distance_sensor_channel": "0"},
            {"tank_floor_value": "0.66"},
            {"tank_ceiling_value": "0.16"},
        ],
    )

    rear_tank_node = Node(
        package="eel",
        executable="tank",
        name="rear_tank",
        parameters=[
            {SIMULATE_PARAM: True},
            {"cmd_topic": "tank_rear/cmd"},
            {"status_topic": "tank_rear/status"},
            {"motor_pin": "24"},
            {"direction_pin": "25"},
            {"distance_sensor_channel": "1"},
            {"tank_floor_value": "0.288"},
            {"tank_ceiling_value": "0.005"},
        ],
    )

    ld.add_action(pressure_node)
    ld.add_action(rudder_node)
    # ld.add_action(depth_pid_control_node)
    # ld.add_action(depth_control_node)
    ld.add_action(imu_node)
    ld.add_action(motor_node)
    ld.add_action(front_tank_node)
    ld.add_action(rear_tank_node)

    return ld
