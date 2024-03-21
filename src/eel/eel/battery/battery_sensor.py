from .battery_utils import calculate_voltage_percent


class BatterySensor:
    def __init__(self):
        import logging
        from ina226 import INA226

        self.ina = INA226(busnum=1, max_expected_amps=16, log_level=logging.INFO)
        self.ina.configure()

    def get_voltage_percent(self):
        voltage = self.ina.voltage()
        percent = calculate_voltage_percent(voltage)
        return float(percent / 100)

    def get_voltage(self):
        return self.ina.voltage()

    def get_current(self):
        return self.ina.current()

    def get_power(self):
        return self.ina.power()

    def get_supply_voltage(self):
        return self.ina.supply_voltage()

    def get_shunt_voltage(self):
        return self.ina.shunt_voltage()


if __name__ == "__main__":
    battery_sensor = BatterySensor()

    print("Bus Voltage    : %.3f V" % battery_sensor.voltage())
    print("Bus Current    : %.3f mA" % battery_sensor.current())
    print("Supply Voltage : %.3f V" % battery_sensor.supply_voltage())
    print("Shunt voltage  : %.3f mV" % battery_sensor.shunt_voltage())
    print("Power          : %.3f mW" % battery_sensor.power())
