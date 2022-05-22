from rclpy.node import Node
from eel_interfaces.msg import TankStatus
from ..utils.topics import FRONT_TANK_STATUS, REAR_TANK_STATUS


class PressureSensorSimulator:
    def __init__(self, parent_node: Node) -> None:
        parent_node.create_subscription(
            TankStatus, FRONT_TANK_STATUS, self._handle_front_tank_msg, 10
        )
        parent_node.create_subscription(
            TankStatus, REAR_TANK_STATUS, self._handle_rear_tank_msg, 10
        )

        self._current_depth = 0.0
        self._front_tank_level = 0.0
        self._rear_tank_level = 0.0

        self.logger = parent_node.get_logger()

    def _handle_front_tank_msg(self, msg):
        self._front_tank_level = msg.current_level
        self._calculate_depth()

    def _handle_rear_tank_msg(self, msg):
        self._rear_tank_level = msg.current_level
        self._calculate_depth()

    def _calculate_depth(self):
        self._current_depth = (
            self._front_tank_level * 10 + self._rear_tank_level * 10
        ) / 2

    def get_current_depth(self):
        return self._current_depth
