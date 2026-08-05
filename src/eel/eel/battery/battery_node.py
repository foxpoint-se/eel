#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node

from eel_interfaces.msg import BatteryStatus

from ..utils.constants import SIMULATE_PARAM
from ..utils.node_runner import spin_node_until_shutdown
from ..utils.topics import BATTERY_STATUS
from .battery_sensor import BatterySensor
from .battery_sim import BatterySimulator


class BatteryNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(BatteryStatus, BATTERY_STATUS, 10)

        # hertz (publications per second)
        self.update_frequency = 2

        sensor: BatterySensor | BatterySimulator
        sensor = BatterySensor() if not self.should_simulate else BatterySimulator(parent_node=self)
        self.get_voltage_v = sensor.get_voltage_v
        self.get_current_a = sensor.get_current_a
        self.get_power_w = sensor.get_power_w
        self.get_supply_voltage_v = sensor.get_supply_voltage_v
        self.get_shunt_voltage_v = sensor.get_shunt_voltage_v
        self.get_voltage_ratio = sensor.get_voltage_ratio

        self.updater = self.create_timer(1.0 / self.update_frequency, self.publish_battery)

        self.get_logger().info("Battery node started")

    def publish_battery(self) -> None:
        msg = BatteryStatus()
        msg.voltage_v = self.get_voltage_v()
        msg.current_a = self.get_current_a()
        msg.power_w = self.get_power_w()
        msg.supply_voltage_v = self.get_supply_voltage_v()
        msg.shunt_voltage_v = self.get_shunt_voltage_v()
        msg.voltage_ratio = self.get_voltage_ratio()

        self.status_publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BatteryNode()
    spin_node_until_shutdown(node)


if __name__ == "__main__":
    main()
