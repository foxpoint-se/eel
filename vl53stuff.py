import time
from tkinter import OFF
import board
import busio

import adafruit_vl53l0x
import RPi.GPIO as GPIO

i2c = busio.I2C(board.SCL, board.SDA)
i2c.scan()

FRONT_XSHUT_PIN = 21
ON_LEVEL = GPIO.HIGH
OFF_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(FRONT_XSHUT_PIN, GPIO.OUT)
print("xshut off")
GPIO.output(FRONT_XSHUT_PIN, OFF_LEVEL)

print("init rear")

vl53_rear = None

try:
    vl53_rear = adafruit_vl53l0x.VL53L0X(i2c)
    print("set rear to 12")
    vl53_rear.set_address(12)
except:
    print("rear already at 12")
    vl53_rear = adafruit_vl53l0x.VL53L0X(i2c, 12)

vl53_rear.measurement_timing_budget = 500000

print("sleep 1")
time.sleep(1)
print("xshut on")
GPIO.output(FRONT_XSHUT_PIN, ON_LEVEL)

print("init front")
vl53_front = adafruit_vl53l0x.VL53L0X(i2c)
vl53_front.measurement_timing_budget = 500000


print("PRINT!")
while True:
    print("rear", vl53_rear.range, "mm", "front", vl53_front.range, "mm")
