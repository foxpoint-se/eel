import argparse
import time
import board
import busio
import adafruit_vl53l0x
import adafruit_bno055


parser = argparse.ArgumentParser()
parser.add_argument("-f", "--front", action="count", help="Reads front distance sensor")
parser.add_argument("-r", "--rear", action="count", help="Reads rear distance sensor")
args = parser.parse_args()


i2c = busio.I2C(board.SCL, board.SDA)
sensor_address = None

if args.rear:
    sensor_address = 29

if args.front:
    sensor_address = 22

vl53 = adafruit_vl53l0x.VL53L0X(i2c, address=sensor_address)
vl53.measurement_timing_budget = 500000
vl53.start_continuous()
print(f"Is vl53 in cont mode {vl53.is_continuous_mode}, on address {sensor_address}")

while True:
    print("range", vl53.range, "mm")
    print("")
    time.sleep(0.3)
