"""Battery logic: BatteryRaw → BatteryStatus (adds voltage_ratio)."""

from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import BatteryRaw, BatteryStatus

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import BATTERY_RAW, BATTERY_STATUS
from .battery_utils import calculate_voltage_ratio


class BatteryNode(Node):
    def __init__(self) -> None:
        super().__init__("battery")
        self.create_subscription(BatteryRaw, BATTERY_RAW, self._handle_raw, 10)
        self._pub = self.create_publisher(BatteryStatus, BATTERY_STATUS, 10)
        self.get_logger().info(f"Battery started (listening on {BATTERY_RAW})")

    def _handle_raw(self, msg: BatteryRaw) -> None:
        out = BatteryStatus()
        out.voltage_v = float(msg.voltage_v)
        out.current_a = float(msg.current_a)
        out.power_w = float(msg.power_w)
        out.supply_voltage_v = float(msg.supply_voltage_v)
        out.shunt_voltage_v = float(msg.shunt_voltage_v)
        out.voltage_ratio = calculate_voltage_ratio(float(msg.voltage_v))
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BatteryNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
