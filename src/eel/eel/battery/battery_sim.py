from time import time

from rclpy.node import Node

from .battery_utils import calculate_voltage_ratio

BATTERY_MAX_VOLTAGE = 16.8
DEGENERATION_TIME_S = 30
DEGEN_RATE_PERCENT = 0.00


class BatterySimulator:
    def __init__(self, parent_node: Node) -> None:
        self.start_time = time()
        self.voltage_v = BATTERY_MAX_VOLTAGE
        self.current_a = 2.56
        self.power_w = self.voltage_v * self.current_a
        self.supply_voltage_v = self.voltage_v
        self.shunt_voltage_v = 0.0

        self.last_update = self.start_time

        self.battery_updater = parent_node.create_timer(1.0, self._loop)

    def _loop(self) -> None:
        self._update_voltage()
        self.last_update = time()

    def _update_voltage(self) -> None:
        now = time()

        if (int(now) - int(self.start_time)) % DEGENERATION_TIME_S == 0:
            self.voltage_v = self.voltage_v - (BATTERY_MAX_VOLTAGE * DEGEN_RATE_PERCENT)
            self.supply_voltage_v = self.voltage_v
            self.power_w = self.voltage_v * self.current_a

    def get_voltage_v(self) -> float:
        return self.voltage_v

    def get_voltage_ratio(self) -> float:
        return calculate_voltage_ratio(self.get_voltage_v())

    def get_current_a(self) -> float:
        return self.current_a

    def get_power_w(self) -> float:
        return self.power_w

    def get_supply_voltage_v(self) -> float:
        return self.supply_voltage_v

    def get_shunt_voltage_v(self) -> float:
        return self.shunt_voltage_v
