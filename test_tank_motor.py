import argparse
from fcntl import F_SEAL_SEAL
from queue import Empty
import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO


parser = argparse.ArgumentParser()
parser.add_argument("-f", "--front", action="count", help="Start front motor")
parser.add_argument("-r", "--rear", action="count", help="Starts rear motor")
parser.add_argument("-t", "--time", type=int, choices=[1, 3, 5, 10, 20], help="Seconds to run")
parser.add_argument("--empty", type=int, choices=[0, 1], help="If tank should empty or fyll 0 = fill 1 = empty")
args = parser.parse_args()

if args.front and args.rear:
    raise ValueError("Cannot run both front and rear at the same time")

if args.rear:
    DEFAULT_PWM_PIN = 24
    DEFAULT_DIR_PIN = 25
if args.front:
    DEFAULT_PWM_PIN = 23
    DEFAULT_DIR_PIN = 18


PWM_FREQUENCY = 4096
FRONT_XSHUT_PIN = 21
ON_LEVEL = GPIO.HIGH
OFF_LEVEL = GPIO.LOW
GPIO.setup(FRONT_XSHUT_PIN, GPIO.OUT)
print("xshut off")
GPIO.output(FRONT_XSHUT_PIN, ON_LEVEL)

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
pwm_output = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)


seconds_to_run = args.time
empty = args.empty


if empty:
    GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
else:
    GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)

before = time.time()

now = before

print("start")
pwm_output.start(60)

while now < before + seconds_to_run:
    now = time.time()

pwm_output.stop()
print("stop")