import argparse
import logging
import time

import RPi.GPIO as GPIO
import adafruit_vl53l0x
import board
import busio

logging.basicConfig(format="%(asctime)s %(message)s", level=logging.DEBUG)

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW


class MotorControl:
    def __init__(self, motor_pin, direction_pin) -> None:
        self.motor_pin = motor_pin
        self.direction_pin = direction_pin
        self.motor_state = False

        # Initialize PI pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.motor_pin, GPIO.OUT)

    def set_forward(self):
        GPIO.output(self.direction_pin, FORWARD_LEVEL)

    def set_backward(self):
        GPIO.output(self.direction_pin, BACKWARD_LEVEL)

    def start(self):
        GPIO.output(self.motor_pin, FORWARD_LEVEL)
        self.motor_state = True

    def stop(self):
        GPIO.output(self.motor_pin, GPIO.LOW)
        self.motor_state = False


class DistanceSensor:

    def __init__(self, address, shut_pin, timing_budget=500000):
        self.address = address
        self.shut_pin = shut_pin
        self.resolution = 0.0  # TODO Find sensor resolution, used to determine safe max/min limit

        # GPIO setup
        GPIO.setup(self.shut_pin, GPIO.OUT)
        self.activate_sensor()

        # Instantiate i2c object
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
        self.vl53.measurement_timing_budget = timing_budget
        self.set_address(self.address)

    @property
    def resolution(self):
        return self.resolution

    @property
    def address(self):
        return self.address

    def set_address(self, address):
        self.vl53.set_address(address)

    def get_range(self):
        return self.vl53.range

    def activate_sensor(self):
        GPIO.output(self.shut_pin, GPIO.HIGH)

    def deactivate_sensor(self):
        GPIO.output(self.shut_pin, GPIO.LOW)


class BallastTank:
    def __init__(self, motor, distance_sensor):
        self._motor = motor
        self._distance_sensor = distance_sensor

        self.current_level = self.get_level()

        # TODO Need to collect the MAX and MIN value for the distance sensor
        self.MAX_DISTANCE = 0  # Max distance in cm for ballast tank
        self.MIN_DISTANCE = 0  # Min distance in cm for ballast tank

        self.ACCEPTED_OFFSET = 1  # Accepted offset for level fill

    def set_level(self, level):
        if not 0.0 <= level <= 100.0:
            raise ValueError("Level can only be between 0 and 100")

        self.current_level = self.get_level()
        self.current_distance = self.get_distance()
        low_threshold = level - self.ACCEPTED_OFFSET
        high_threshold = level + self.ACCEPTED_OFFSET
        logging_interval = 2.0
        logging_ts = 0.0

        fill = level > self.current_level
        empty = level < self.current_level
        max_level = self.MAX_DISTANCE - self._distance_sensor.resolution
        min_level = self.MIN_DISTANCE + self._distance_sensor.resolution

        if fill:
            self._motor.set_forward()

        if empty:
            self._motor.set_backward()

        logging.info(f"Wanted level {level}, starting motor")
        self._motor.start()
        while not low_threshold < self.current_level < high_threshold:
            self.current_level = self.get_level()
            self.current_distance = self.get_distance()

            # Give logging ouput to see progress
            current_time = time.time()
            if current_time > logging_ts + logging_interval:
                logging.info(f"Current level is: {self.current_level}, percent left: {abs(self.current_level - level)}")
                logging_ts = current_time

            # Check if critical values are reached
            if self.current_distance <= min_level or self.current_distance >= max_level:
                logging.warning(f"Level reached a unwanted value: {self.current_level}")
                break

        logging.info(f"Ballast level is now {self.current_level} stopping motor.")
        self._motor.stop()

    def get_level(self):
        distance = self.get_distance()
        level = distance / self.MAX_DISTANCE

        return level

    def get_distance(self):
        return self._distance_sensor.get_range()


if __name__ == "__main__":
    DEFAULT_MOTOR_PIN = 22  # 27
    DEFAULT_DIR_PIN = 23  # 24
    DEFAULT_SENSOR_ADDRESS = 56
    DEFAULT_SHUT_PIN = 20

    motor = MotorControl(DEFAULT_MOTOR_PIN, DEFAULT_DIR_PIN)
    dist_sensor = DistanceSensor(DEFAULT_SENSOR_ADDRESS, DEFAULT_SHUT_PIN)
    ballast_tank = BallastTank(motor, dist_sensor)

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--", type=float, default=0.0, help="Sets the ballast tank fill level, given as percentage"
    )
    args = parser.parse_args()

    fill_level = args.set
    ballast_tank.set_level(fill_level)
