import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_pca9685

DEFAULT_DIR_PIN = 25

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)

i2c = busio.I2C(board.SCL, board.SDA)

pca = adafruit_pca9685.PCA9685(i2c, address=0x40)
pca.frequency = 1000

pca.channels[0].duty_cycle = 0x0
