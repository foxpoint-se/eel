from .battery_utils import calculate_voltage_ratio


class BatterySensor:
    def __init__(self) -> None:
        import logging

        from ina226 import INA226

        self.ina = INA226(busnum=1, max_expected_amps=16, log_level=logging.INFO)
        self.ina.configure()

    def get_voltage_ratio(self) -> float:
        return calculate_voltage_ratio(self.ina.voltage())

    def get_voltage_v(self) -> float:
        return float(self.ina.voltage())

    def get_current_a(self) -> float:
        return float(self.ina.current()) / 1000.0

    def get_power_w(self) -> float:
        return float(self.ina.power()) / 1000.0

    def get_supply_voltage_v(self) -> float:
        return float(self.ina.supply_voltage())

    def get_shunt_voltage_v(self) -> float:
        return float(self.ina.shunt_voltage()) / 1000.0
