from .battery_utils import calculate_voltage_percent


class BatterySensor:
    def __init__(self):
        import logging
        from ina226 import INA226

        self.ina = INA226(busnum=1, max_expected_amps=16, log_level=logging.INFO)
        self.ina.configure()

    # TODO: not percent. change name to get_voltage_ratio
    def get_voltage_percent(self):
        voltage = self.ina.voltage()
        percent = calculate_voltage_percent(voltage)
        return float(percent / 100)

    # NOTE: unit is V
    # TODO: include unit in name
    def get_voltage(self):
        return self.ina.voltage()

    # NOTE: unit is mA
    # TODO: include unit in name
    def get_current(self):
        return self.ina.current()

    # NOTE: unit is mW
    # TODO: divide by 1000 so that unit is W
    # TODO: include unit in name
    def get_power(self):
        return self.ina.power()

    # NOTE: unit is V
    # TODO: include unit in name
    def get_supply_voltage(self):
        return self.ina.supply_voltage()

    # NOTE: unit is V
    # TODO: include unit in name
    def get_shunt_voltage(self):
        return self.ina.shunt_voltage()
