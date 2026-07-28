import os
import unittest

import launch
import pytest
from ament_index_python.packages import get_package_share_directory
from eel.utils.topics import (
    BATTERY_STATUS,
    DEPTH_CONTROL_STATUS,
    FRONT_TANK_STATUS,
    IMU_STATUS,
    LEAKAGE_STATUS,
    MODEM_STATUS,
    NAVIGATION_STATUS,
    PRESSURE_STATUS,
    REAR_TANK_STATUS,
    RUDDER_STATUS,
)
from geometry_msgs.msg import Vector3
from launch.actions import IncludeLaunchDescription
from launch.events.process.process_exited import ProcessExited
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from launch_testing_ros import WaitForTopics
from std_msgs.msg import Bool

from eel_interfaces.msg import (
    BatteryStatus,
    DepthControlStatus,
    ImuStatus,
    ModemStatus,
    NavigationStatus,
    PressureStatus,
    TankStatus,
)

TOPIC_TIMEOUT_SEC = 20.0

# Sim nodes that publish a status topic we can wait on at startup.
STATUS_PUBLISHERS: list[tuple[str, str, type]] = [
    ("imu_node", IMU_STATUS, ImuStatus),
    ("battery_node", BATTERY_STATUS, BatteryStatus),
    ("pressure_node", PRESSURE_STATUS, PressureStatus),
    ("front_tank", FRONT_TANK_STATUS, TankStatus),
    ("rear_tank", REAR_TANK_STATUS, TankStatus),
    ("leakage_node", LEAKAGE_STATUS, Bool),
    ("modem_node", MODEM_STATUS, ModemStatus),
    ("rudder_node", RUDDER_STATUS, Vector3),
    ("depth_control_rudder_node", DEPTH_CONTROL_STATUS, DepthControlStatus),
    ("navigation_action_client", NAVIGATION_STATUS, NavigationStatus),
]


def _assert_no_process_crashed(proc_info) -> None:
    for event in proc_info:
        if isinstance(event, ProcessExited) and event.returncode != 0:
            name = getattr(event.action, "name", str(event.action))
            raise AssertionError(f"{name} exited with code {event.returncode}")


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory("eel_bringup"),
        "launch",
        "sim_all_nodes_startup.launch.py",
    )
    return (
        launch.LaunchDescription(
            [
                IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
                ReadyToTest(),
            ]
        ),
        {},
    )


class TestAllSimNodesPublishStatusOnStartup(unittest.TestCase):
    def test__when_all_sim_nodes_start__should_publish_status_on_startup(self, proc_info) -> None:
        for _node_name, topic, msg_type in STATUS_PUBLISHERS:
            with WaitForTopics([(topic, msg_type)], timeout=TOPIC_TIMEOUT_SEC):
                pass
            _assert_no_process_crashed(proc_info)

        _assert_no_process_crashed(proc_info)
