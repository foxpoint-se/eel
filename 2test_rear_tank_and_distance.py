import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

PWM_FREQUENCY = 1000
DEFAULT_PWM_PIN = 24
DEFAULT_DIR_PIN = 25

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
pwm_output = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)

i2c = busio.I2C(board.SCL, board.SDA)

vl53 = adafruit_vl53l0x.VL53L0X(i2c)

vl53.measurement_timing_budget = 500000

seconds_to_run = 4
go_backwards = True

if go_backwards:
    GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
else:
    GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)

before = time.time()

now = before

print("start")
pwm_output.start(80)

while now < before + seconds_to_run:
    print("range", vl53.range, "mm")
    now = time.time()

pwm_output.stop()
print("stop")