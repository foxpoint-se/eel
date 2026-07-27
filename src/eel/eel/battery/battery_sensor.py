from .battery_utils import calculate_voltage_percent


class BatterySensor:
    def __init__(self) -> None:
        import logging

        from ina226 import INA226

        self.ina = INA226(busnum=1, max_expected_amps=16, log_level=logging.INFO)
        self.ina.configure()

    # TODO: not percent. change name to get_voltage_ratio
    def get_voltage_percent(self) -> float:
        voltage = self.ina.voltage()
        percent = calculate_voltage_percent(voltage)
        return float(percent / 100)

    # NOTE: unit is V
    # TODO: include unit in name
    def get_voltage(self) -> float:
        return float(self.ina.voltage())

    # NOTE: unit is mA
    # TODO: include unit in name
    def get_current(self) -> float:
        return float(self.ina.current())

    # NOTE: unit is mW
    # TODO: divide by 1000 so that unit is W
    # TODO: include unit in name
    def get_power(self) -> float:
        return float(self.ina.power())

    # NOTE: unit is V
    # TODO: include unit in name
    def get_supply_voltage(self) -> float:
        return float(self.ina.supply_voltage())

    # NOTE: unit is V
    # TODO: include unit in name
    def get_shunt_voltage(self) -> float:
        return float(self.ina.shunt_voltage())
