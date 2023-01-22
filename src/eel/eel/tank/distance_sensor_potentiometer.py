import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from .distance_sensor_base import DistanceSensorBase


POTENTIOMETER_FLOOR = 0
POTENTIOMETER_CEILING = 26600

PISTON_FLOOR_MM = 15
PISTON_CEILING_MM = 63


def translate_from_range_to_range(value, from_min, from_max, to_min, to_max):
    # Figure out how 'wide' each range is
    left_span = from_max - from_min
    right_span = to_max - to_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - from_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return to_min + (value_scaled * right_span)


def cap_value(value):
    if value > POTENTIOMETER_CEILING:
        return POTENTIOMETER_CEILING
    elif value < POTENTIOMETER_FLOOR:
        return POTENTIOMETER_FLOOR
    return value


class DistanceSensorPotentiometer(DistanceSensorBase):
    def __init__(self) -> None:
        super().__init__()
        self.__i2c = busio.I2C(board.SCL, board.SDA)
        self.__ads = ADS.ADS1015(self.__i2c)
        # TODO: make the pin dynamic
        self.__channel = AnalogIn(self.__ads, ADS.P1)

    def __get_raw_value(self):
        return self.__channel.value

    def __get_pretty_value(self):
        raw = self.__get_raw_value()
        capped = cap_value(raw)
        to_mm = translate_from_range_to_range(
            capped,
            POTENTIOMETER_FLOOR,
            POTENTIOMETER_CEILING,
            PISTON_FLOOR_MM,
            PISTON_CEILING_MM,
        )
        return to_mm

    def get_range(self) -> float:
        return self.__get_pretty_value()
