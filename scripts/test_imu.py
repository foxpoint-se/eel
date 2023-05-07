import adafruit_bno055
import board
import time


i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)


while True:
    heading, roll, pitch = sensor.euler
    print(f"Heading: {heading}\tRoll: {roll}\tPith: {pitch}")
    time.sleep(1)