#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

from eel_interfaces.msg import NavigationStatus

from ..utils.constants import SIMULATE_PARAM, NavigationMissionStatus
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import NAVIGATION_STATUS
from .led_pulse import PulseStep, plan_pulse_steps
from .led_sim import LEDBackend, LEDSimulator

# Every SEQUENCE_PERIOD_S the LED plays one pattern for the current mission status.
# Each character ≈ 0.1s. '#' = on, ' ' = off (gap between pulses is essentially instant).
#
# waiting    2×0.50s   ##### #####................
# acquired   3×0.25s   ## ## ##...................
# started    4×0.10s   # # # #....................
# cancelled  1×1.50s   ###############............
# finished   6×0.25s   ## ## ## ## ## ##..........
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


class LED(Node):
    def __init__(self) -> None:
        super().__init__("LED_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = bool(self.get_parameter(SIMULATE_PARAM).value)

        self._led_control = None
        if self.should_simulate:
            self.led_control: LEDBackend = LEDSimulator()
        else:
            from .led_control import LEDControl

            self._led_control = LEDControl()
            self.led_control = self._led_control

        self.navigation_status_subscription = self.create_subscription(
            NavigationStatus, NAVIGATION_STATUS, self.update_mission_status, 10
        )

        self.navigation_status = NavigationMissionStatus.WAITING_FOR_MISSION.value
        self._pulse_steps: list[PulseStep] = []
        self._pulse_timer: Timer = self.create_timer(1.0, self._on_pulse_timer)
        self._pulse_timer.cancel()
        self.create_timer(SEQUENCE_PERIOD_S, self.pulse_led)

        self.get_logger().info(f"{'SIMULATE ' if self.should_simulate else ''}LED node started.")

    def update_mission_status(self, msg: NavigationStatus) -> None:
        self.navigation_status = msg.mission_status

    def pulse_led(self) -> None:
        if self._pulse_steps:
            return

        nof_pulses = PULSE_COUNT_MAP.get(self.navigation_status)
        pulse_length = PULSE_TIME_MAP.get(self.navigation_status)
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
        if action == "on":
            self.led_control.on()
        else:
            self.led_control.off()

        if delay_s > 0:
            self._pulse_timer.timer_period_ns = int(delay_s * 1e9)
            self._pulse_timer.reset()
        else:
            self._run_pulse_step()

    def shutdown(self) -> None:
        self._pulse_timer.cancel()
        self._pulse_steps = []
        self.led_control.off()
        if self._led_control is not None:
            self._led_control.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LED()
    spin_node_until_shutdown(node, cleanup=node.shutdown)


if __name__ == "__main__":
    main()
