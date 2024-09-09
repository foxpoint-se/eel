#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from .led_control import LEDControl
from ..utils.topics import NAVIGATION_STATUS
from ..utils.constants import NavigationMissionStatus

from eel_interfaces.msg import NavigationStatus


PULSE_COUNT_MAP = {
    NavigationMissionStatus.WAITING_FOR_MISSION: 2,
    NavigationMissionStatus.MISSION_AQUIRED: 3,
    NavigationMissionStatus.MISSION_STARTED: 4,
    NavigationMissionStatus.MISSION_CANCELLED: 1,
    NavigationMissionStatus.MISSION_FINISHED: 6
}

PULSE_TIME_MAP = {
    NavigationMissionStatus.WAITING_FOR_MISSION: 0.5,
    NavigationMissionStatus.MISSION_AQUIRED: 0.25,
    NavigationMissionStatus.MISSION_STARTED: 0.1,
    NavigationMissionStatus.MISSION_CANCELLED: 1.5,
    NavigationMissionStatus.MISSION_FINISHED: 0.25

}


class LED(Node):
    def __init__(self):
        super().__init__("LED_node")

        self.led_control = LEDControl()

        self.navigation_status_subscription = self.create_subscription(
            NavigationStatus, NAVIGATION_STATUS, self.update_mission_status, 10
        )

        self.poller = self.create_timer(3.0, self.pulse_led)
        self.navigation_status = NavigationMissionStatus.WAITING_FOR_MISSION

        self.get_logger().info("Started LED node")

    def update_mission_status(self, msg):
        self.navigation_status = msg.mission_status

    def pulse_led(self):
        nof_pulses = PULSE_COUNT_MAP.get(self.navigation_status)
        pulse_length = PULSE_TIME_MAP.get(self.navigation_status)

        self.led_control.sequence(nof_pulses, pulse_length)
    

def main(args=None):
    rclpy.init(args=args)
    node = LED()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()