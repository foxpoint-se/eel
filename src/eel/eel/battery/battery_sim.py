from rclpy.node import Node
from time import time
from .battery_utils import calculate_voltage_percent

BATTERY_MAX_VOLTAGE = 16.8
DEGENERATION_TIME_S = 30
DEGEN_RATE_PERCENT = 0.00


class BatterySimulator:
    def __init__(self, parent_node: Node) -> None:
        self.start_time = time()
        self.voltage = BATTERY_MAX_VOLTAGE
        self.current = 2.56
        self.power = self.voltage * self.current
        self.supply_voltage = self.voltage
        self.shunt_voltage = self.voltage

        self.last_update = self.start_time

        self.battery_updater = parent_node.create_timer(1.0, self._loop)

    def _loop(self):
        self._update_voltage()
        self.last_update = time()

    def _update_voltage(self):
        now = time()

        if (int(now) - int(self.start_time)) % DEGENERATION_TIME_S == 0:
            self.voltage = self.voltage - (BATTERY_MAX_VOLTAGE * DEGEN_RATE_PERCENT)

    def get_voltage(self):
        return self.voltage

    def get_voltage_percent(self):
        voltage = self.get_voltage()
        percent = calculate_voltage_percent(voltage)
        return float(percent / 100)

    def get_current(self):
        return self.current

    def get_power(self):
        return self.power

    def get_supply_voltage(self):
        return self.supply_voltage

    def get_shunt_voltage(self):
        return self.shunt_voltage
