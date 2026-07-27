from rclpy.node import Node

from eel_interfaces.msg import PressureStatus

from ..utils.topics import PRESSURE_STATUS
from .modem_source import ModemSource


class ModemSimulator(ModemSource):
    def __init__(self, parent_node: Node) -> None:

        self._current_depth = 0.0
        parent_node.create_subscription(PressureStatus, PRESSURE_STATUS, self._handle_pressure_status_msg, 10)

    def _handle_pressure_status_msg(self, msg: PressureStatus) -> None:
        self._current_depth = msg.depth

    def get_received_signal_strength_indicator(self) -> int | None:
        if self._current_depth < 0.1:
            signal_strength = 31
        elif 0.1 < self._current_depth < 0.2:
            signal_strength = 18
        else:
            signal_strength = 0

        return signal_strength

    def get_registration_status(self) -> int | None:
        registration_status = 1 if self._current_depth < 0.2 else 0

        return registration_status

    def ping(self) -> bool:
        if self._current_depth < 0.2:
            connectivity = True
        else:
            connectivity = False

        return connectivity
