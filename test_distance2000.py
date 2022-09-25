import adafruit_vl53l0x
import adafruit_pca9685
import time
import board
import busio
import RPi.GPIO as GPIO

DEFAULT_DIR_PIN = 25

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)

i2c = busio.I2C(board.SCL, board.SDA)

pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 100
pca.channels[0].duty_cycle = 0x0

vl53 = adafruit_vl53l0x.VL53L0X(i2c)
vl53.measurement_timing_budget = 20000


go_backwards = True
if go_backwards:
    GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
else:
    GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)


seconds_to_run = 2
before = time.time()
now = before
print("start")
pca.channels[0].duty_cycle = 0xB8D3
while now < before + seconds_to_run:
    now = time.time()
    try:
        print("range", vl53.range, "mm")
    except Exception as err:
        print("stopping. error", err)
        pca.channels[0].duty_cycle = 0x0


print("stop")
pca.channels[0].duty_cycle = 0x0
