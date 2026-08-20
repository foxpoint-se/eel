"""LED logic: nav status → pulse pattern → led/setpoint."""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Bool

from eel_interfaces.msg import NavigationStatus

from ..utils.constants import NavigationMissionStatus
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import LED_SETPOINT, NAVIGATION_STATUS
from .led_pulse import PulseStep, plan_pulse_steps

PULSE_COUNT_MAP = {
    NavigationMissionStatus.WAITING_FOR_MISSION.value: 2,
    NavigationMissionStatus.MISSION_AQUIRED.value: 3,
    NavigationMissionStatus.MISSION_STARTED.value: 4,
    NavigationMissionStatus.MISSION_CANCELLED.value: 1,
    NavigationMissionStatus.MISSION_FINISHED.value: 6,
}

PULSE_TIME_MAP = {
    NavigationMissionStatus.WAITING_FOR_MISSION.value: 0.5,
    NavigationMissionStatus.MISSION_AQUIRED.value: 0.25,
    NavigationMissionStatus.MISSION_STARTED.value: 0.1,
    NavigationMissionStatus.MISSION_CANCELLED.value: 1.5,
    NavigationMissionStatus.MISSION_FINISHED.value: 0.25,
}

SEQUENCE_PERIOD_S = 3.0


class LedNode(Node):
    def __init__(self) -> None:
        super().__init__("led")
        self._setpoint_pub = self.create_publisher(Bool, LED_SETPOINT, 10)
        self.create_subscription(NavigationStatus, NAVIGATION_STATUS, self._handle_nav, 10)
        self._navigation_status = NavigationMissionStatus.WAITING_FOR_MISSION.value
        self._pulse_steps: list[PulseStep] = []
        self._pulse_timer: Timer = self.create_timer(1.0, self._on_pulse_timer)
        self._pulse_timer.cancel()
        self.create_timer(SEQUENCE_PERIOD_S, self._pulse_led)
        self.get_logger().info("LED started (publishing led/setpoint)")

    def _publish_setpoint(self, on: bool) -> None:
        msg = Bool()
        msg.data = on
        self._setpoint_pub.publish(msg)

    def _handle_nav(self, msg: NavigationStatus) -> None:
        self._navigation_status = msg.mission_status

    def _pulse_led(self) -> None:
        if self._pulse_steps:
            return
        nof_pulses = PULSE_COUNT_MAP.get(self._navigation_status)
        pulse_length = PULSE_TIME_MAP.get(self._navigation_status)
        if nof_pulses is None or pulse_length is None:
            return
        self._pulse_steps = plan_pulse_steps(nof_pulses, pulse_length)
        self._run_pulse_step()

    def _on_pulse_timer(self) -> None:
        self._run_pulse_step()

    def _run_pulse_step(self) -> None:
        if not self._pulse_steps:
            self._pulse_timer.cancel()
            return
        action, delay_s = self._pulse_steps.pop(0)
        self._publish_setpoint(action == "on")
        if delay_s > 0:
            self._pulse_timer.timer_period_ns = int(delay_s * 1e9)
            self._pulse_timer.reset()
        else:
            self._run_pulse_step()

    def shutdown(self) -> None:
        self._pulse_timer.cancel()
        self._pulse_steps = []
        self._publish_setpoint(False)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LedNode()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
