#!/usr/bin/python
import ms5837
import time

sensor = ms5837.MS5837_30BA()  # Default I2C bus is 1 (Raspberry Pi 3)
# We must initialize the sensor before reading it
if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

print("pressure {} ".format(sensor.pressure(ms5837.UNITS_atm)))


print("temp {}".format(sensor.temperature(ms5837.UNITS_Centigrade)))


freshwaterDepth = sensor.depth()  # default is freshwater
sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
saltwaterDepth = sensor.depth()  # No nead to read() again
# sensor.setFluidDensity(1000)  # kg/m^3

calibration_factor = 0.19

print("hold at sea level")

time.sleep(7)

sensor.read()
sea_level_value = sensor.depth()

print("registered sea level", sea_level_value)
print("===")
print("hold at 19 cm")

time.sleep(7)

sensor.read()
calibration_value = sensor.depth()
print("registered calibration value at", calibration_factor, "m:", calibration_value)

time.sleep(3)

# Spew readings
while True:
    if sensor.read():

        current_reading = sensor.depth()

        our_depth = (
            (current_reading - sea_level_value)
            / (calibration_value - sea_level_value)
            * calibration_factor
        )

        # print("P: %0.1f mbar  %0.3f psi\tT: %0.2f C  %0.2f F") % (
        print(
            "pressure mbar",
            round(sensor.pressure(), 2),
            "\t",
            "depth m",
            round(freshwaterDepth, 4),
            "\t",
            round(sensor.temperature(), 2),
            "\t",
            round(our_depth, 4),
        )  # Default is mbar (no arguments)
    else:
        print("Sensor read failed!")
        exit(1)
