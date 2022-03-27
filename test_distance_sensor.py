import time
import board
import busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)

vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# vl53.measurement_timing_budget = 200000
vl53.measurement_timing_budget = 500000

while True:
    print("range", vl53.range, "mm")
    time.sleep(1.0)

# average_count = 10
# measurements = []
# while True:
#     if len(measurements) < average_count:
#         measurements.append()
