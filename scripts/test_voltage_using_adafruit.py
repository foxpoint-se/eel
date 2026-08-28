import time
import board
import adafruit_ina260

# TODO:
# - [ ] On Ålen: pip3 install adafruit-circuitpython-ina260
# - [ ] See if this script works: python3 scripts/test_voltage_using_adafruit.py
# If it does:
# - [ ] Reimplement battery_sensor.py to use this lib instead
# - [ ] Dockerfile: pip3 install adafruit-circuitpython-ina260
# - [ ] requirements.txt: pip3 install adafruit-circuitpython-ina260
# - [ ] Remove `install-voltage-sensor` from Makefile
# - [ ] Remove ina226 setup from Dockerfile
# - [ ] Remove all occurences of ina226

i2c = board.I2C()
ina260 = adafruit_ina260.INA260(i2c)
while True:
    print("Current: %.2f Voltage: %.2f Power:%.2f"
        %(ina260.current, ina260.voltage, ina260.power))
    time.sleep(1)