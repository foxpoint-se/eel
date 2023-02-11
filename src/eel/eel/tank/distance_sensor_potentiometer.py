import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from .distance_sensor_base import DistanceSensorBase


# NOTE: range when using full range of potentiometer
# floor = 0
# ceiling = 26600


def translate_from_range_to_range(value, from_min, from_max, to_min, to_max):
    # Figure out how 'wide' each range is
    left_span = from_max - from_min
    right_span = to_max - to_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - from_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return to_min + (value_scaled * right_span)


def cap_value(value, floor, ceiling):
    if value > ceiling:
        return ceiling
    elif value < floor:
        return floor
    return value


class DistanceSensorPotentiometer(DistanceSensorBase):
    def __init__(self, floor=5000, ceiling=17000, pin=None) -> None:
        super().__init__()
        if pin == 1:
            self.pin = ADS.P1
        elif pin == 0:
            self.pin = ADS.P0
        else:
            raise Exception("Invalid pin number for potentiometer", pin)
        self.floor = floor
        self.ceiling = ceiling
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__ads = ADS.ADS1015(self.__i2c)

        # NOTE: front tank uses P1. rear uses P0
        self.__channel = AnalogIn(self.__ads, self.pin)

    def __get_raw_value(self):
        return self.__channel.value

    def __get_pretty_value(self):
        raw = self.__get_raw_value()
        capped = cap_value(raw, self.floor, self.ceiling)
        level = translate_from_range_to_range(
            capped,
            self.floor,
            self.ceiling,
            0.0,
            1.0,
        )
        return level

    def get_level(self) -> float:
        return self.__get_pretty_value()
