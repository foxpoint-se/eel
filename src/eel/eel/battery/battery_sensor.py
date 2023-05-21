percent_voltage_map = {
    100: 16.8,
    95: 16.6,
    90: 16.45,
    85: 16.33,
    80: 16.09,
    75: 15.93,
    70: 15.81,
    65: 15.66,
    60: 15.5,
    55: 15.42,
    50: 15.34,
    45: 15.26,
    40: 15.18,
    35: 15.14,
    30: 15.06,
    25: 14.99,
    20: 14.91,
    15: 14.83,
    10: 14.75,
    5: 14.43,
    0: 13.09
}

def calculate_voltage_percent(voltage: float) -> float:
    percent = 0.0

    if voltage >= percent_voltage_map[100]:
        percent = 100.0
        return percent
    
    for mapped_percent, mapped_voltage in percent_voltage_map.items():
        if voltage < mapped_voltage:
            continue
        else:
            percent = mapped_percent
            break

    return percent

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

def read():
    print("Bus Voltage    : %.3f V" % ina.voltage())
    print("Bus Current    : %.3f mA" % ina.current())
    print("Supply Voltage : %.3f V" % ina.supply_voltage())
    print("Shunt voltage  : %.3f mV" % ina.shunt_voltage())
    print("Power          : %.3f mW" % ina.power())


if __name__ == "__main__":
    ina = INA226(busnum=1, max_expected_amps=16, log_level=logging.INFO)
    ina.configure()
    # ina.set_low_battery(5)
    sleep(3)
    print("===================================================Begin to read")
    # read()
    # sleep(2)
    """
    print("===================================================Begin to reset")
    ina.reset()
    sleep(5)
    ina.configure()
    ina.set_low_battery(3)
    sleep(5)
    print("===================================================Begin to sleep")
    ina.sleep()
    sleep(2)
    print("===================================================Begin to wake")
    ina.wake()
    sleep(0.2)
    print("===================================================Read again")
    read()
    sleep(5)
    print("===================================================Trigger test")
    """
    # ina.wake(3)
    # sleep(0.2)
    while True:
        # ina.wake(3)
        # sleep(3)
        while 1:
            if ina.is_conversion_ready():
                sleep(3)
                print(
                    "===================================================Conversion ready"
                )
                read()
                break
        sleep(1)
        print("===================================================Trigger again")
