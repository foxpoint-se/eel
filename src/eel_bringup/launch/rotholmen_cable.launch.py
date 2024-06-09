from launch import LaunchDescription

import os
import sys

# NOTE: workaround to make relative imports work
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from common import (
    create_rudder_node,
    create_battery_node,
    create_gnss_node,
    create_localization_node,
    create_motor_node,
    create_tank_node,
    create_navigation_node,
    create_navigation_client_node,
)


# NOTE: Things to start manually:
# ros2 run eel depth_control_rudder
# ros2 run eel imu
# ros2 run eel pressure
def generate_launch_description():
    ld = LaunchDescription()

    rudder_node = create_rudder_node()
    gnss_node = create_gnss_node()
    # battery_node = create_battery_node()
    localization_node = create_localization_node()
    motor_node = create_motor_node()
    front_tank_node = create_tank_node("front_tank")
    rear_tank_node = create_tank_node("rear_tank")
    navigation_node = create_navigation_node()
    navigation_client_node = create_navigation_client_node()

    ld.add_action(rudder_node)
    ld.add_action(motor_node)
    ld.add_action(localization_node)
    ld.add_action(gnss_node)
    # ld.add_action(battery_node)
    ld.add_action(front_tank_node)
    ld.add_action(rear_tank_node)
    ld.add_action(navigation_node)
    ld.add_action(navigation_client_node)

    return ld
