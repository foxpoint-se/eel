from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


SIMULATE_PARAM = "simulate"


def generate_launch_description():
    ld = LaunchDescription()

    should_simulate_arg = DeclareLaunchArgument(
        SIMULATE_PARAM,
        default_value="false",
        description="Set to false to disable hardware. However, you'll have to run `make serial-sim` to make radio node work.",
    )

    gnss_node = Node(
        package="eel",
        executable="gnss",
        name="gnss_node",
        parameters=[{SIMULATE_PARAM: LaunchConfiguration(SIMULATE_PARAM)}],
    )

    imu_node = Node(
        package="eel",
        executable="imu",
        name="imu_node",
        parameters=[{SIMULATE_PARAM: LaunchConfiguration(SIMULATE_PARAM)}],
    )

    radio_node = Node(
        package="eel",
        executable="radio",
        name="radio_node",
        parameters=[{SIMULATE_PARAM: LaunchConfiguration(SIMULATE_PARAM)}],
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

    ld.add_action(should_simulate_arg)
    ld.add_action(gnss_node)
    ld.add_action(imu_node)
    ld.add_action(radio_node)
    ld.add_action(rudder_node)
    ld.add_action(motor_node)

    return ld
