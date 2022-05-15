import RPi.GPIO as GPIO
import adafruit_vl53l0x
import board
import busio


class DistanceSensor:
    def __init__(self, address, timing_budget=500000):
        self.address = address
        self.resolution = (
            0.0  # TODO Find sensor resolution, used to determine safe max/min limit
        )

        # Instantiate i2c object
        i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
        self.vl53.measurement_timing_budget = timing_budget

    @property
    def resolution(self):
        return self.resolution

    def set_address(self):
        pass

    def get_range(self):
        return self.vl53.range
