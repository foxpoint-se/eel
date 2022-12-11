class BatterySensor:
    def __init__(self):
        import logging
        from ina226 import INA226

        self.ina = INA226(busnum=1, max_expected_amps=16, log_level=logging.INFO)
        self.ina.configure()

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
