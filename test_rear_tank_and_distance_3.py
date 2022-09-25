import time
import board
import busio

import RPi.GPIO as GPIO
import adafruit_pca9685
import adafruit_vl53l0x
import adafruit_bno055

DEFAULT_DIR_PIN = 25

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)

i2c = busio.I2C(board.SCL, board.SDA)

pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 100

vl53 = adafruit_vl53l0x.VL53L0X(i2c)
vl53.measurement_timing_budget = 500000

sensor = adafruit_bno055.BNO055_I2C(i2c)
# vl53.start_continuous()

pca.channels[0].duty_cycle = 0x0
go_backwards = False

if go_backwards:
    GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
else:
    GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)


start_time = time.time()
now = start_time
distance_sample_rate = 1
last_distance_sample = start_time
drive_time = 3

pca.channels[0].duty_cycle = 0xB8D3
while ((now - start_time) < drive_time):
    if (now - last_distance_sample) > distance_sample_rate:
        heading, roll, pitch = sensor.euler
        print(float(heading or 0), float(roll or 0), float(pitch or 0))
        # pca.channels[0].duty_cycle = 0x0
        # print("Will print here...")
        # print("range", vl53.range, "mm")        
        # pca.channels[0].duty_cycle = 0xB8D3
        last_distance_sample = now

    now = time.time()

pca.channels[0].duty_cycle = 0x0
