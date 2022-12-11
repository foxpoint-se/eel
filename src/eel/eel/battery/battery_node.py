#!/usr/bin/env python3
import rlcpy
from rclpy.node import Node
from eel_interfaces.msg import BatteryStatus
from .battery_sensor import BatterySensor
from .battery_sim import BatterySimulator
from ..utils.constants import SIMULATE_PARAM
from ..utils.topics import BATTERY_STATUS


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.declare_parameter(SIMULATE_PARAM, False)
        self.should_simulate = self.get_parameter(SIMULATE_PARAM).value
        self.status_publisher = self.create_publisher(BatteryStatus, BATTERY_STATUS, 10)

        # hertz (publications per second)
        self.update_frequency = 2

        sensor = BatterySensor() if not self.should_simulate else BatterySimulator
        self.get_voltage = sensor.get_voltage
        self.get_current = sensor.get_current
        self.get_power = sensor.get_power
        self.get_supply_voltage = sensor.get_supply_voltage
        self.get_shunt_voltage = sensor.get_shunt_voltage

        self.updater = self.create_timer(1.0 / self.update_frequency, self.publish_battery)

        self.get_logger().info("Battery node started")

    def publish_battery(self):
        msg = BatteryStatus()
        msg.voltage = self.get_voltage()
        msg.current = self.get_current()
        msg.power = self.get_power()
        msg.supply_voltage = self.get_supply_voltage()
        msg.shunt_voltage = self.get_shunt_voltage()

        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
