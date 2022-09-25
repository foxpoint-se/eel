import time
import board
import busio
import adafruit_vl53l0x
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)

vl53 = adafruit_vl53l0x.VL53L0X(i2c)
sensor = adafruit_bno055.BNO055_I2C(i2c)


# vl53.measurement_timing_budget = 200000
vl53.measurement_timing_budget = 500000
vl53.start_continuous()
print(f"Is vl53 in cont mode {vl53.is_continuous_mode}")

while True:
    heading, roll, pitch = sensor.euler
    print(float(heading or 0), float(roll or 0), float(pitch or 0))

    print("range", vl53.range, "mm")
    print("")
    time.sleep(1.0)

# average_count = 10
# measurements = []
# while True:
#     if len(measurements) < average_count:
#         measurements.append()