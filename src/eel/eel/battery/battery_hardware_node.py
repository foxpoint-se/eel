"""Pure battery I/O: INA226 → battery/raw."""

from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import BatteryRaw

from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import BATTERY_RAW
from .battery_sensor import BatterySensor

PUBLISH_HZ = 2.0


class BatteryHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_hardware")
        self._sensor = BatterySensor()
        self._pub = self.create_publisher(BatteryRaw, BATTERY_RAW, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_raw)
        self.get_logger().info("Battery hardware started (publishing battery/raw)")

    def _publish_raw(self) -> None:
        out = BatteryRaw()
        out.voltage_v = self._sensor.get_voltage_v()
        out.current_a = self._sensor.get_current_a()
        out.power_w = self._sensor.get_power_w()
        out.supply_voltage_v = self._sensor.get_supply_voltage_v()
        out.shunt_voltage_v = self._sensor.get_shunt_voltage_v()
        self._pub.publish(out)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BatteryHardwareNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
