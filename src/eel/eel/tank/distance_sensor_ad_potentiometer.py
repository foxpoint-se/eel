#!/usr/bin/python3

import busio
import digitalio
import board

from adafruit_mcp3xxx.analog_in import AnalogIn
from gpiozero import MCP3208


# NOTE: range when using full range of potentiometer
# floor = 1.0
# ceiling = 0.0


def translate_from_range_to_range(value, from_min, from_max, to_min, to_max):
    # Figure out how 'wide' each range is
    left_span = from_max - from_min
    right_span = to_max - to_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - from_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return to_min + (value_scaled * right_span)


def cap_value(value, floor, ceiling):
    if value > max([ceiling, floor]):
        return max([ceiling, floor])
    elif value < min([ceiling, floor]):
        return min([ceiling, floor])
    return value

# channel 0 is front
# channel 1 is rear

# range front: floor 0.66, ceiling 0.16
# range rear: floor 0.288, ceiling 0.005
class DistanceSensorADPotentiometer:
    def __init__(self, floor=None, ceiling=None, channel=None) -> None:
        super().__init__()
        self.channel = channel

        if floor is None or ceiling is None:
            raise Exception("Floor or ceiling not set: Floor, ceiling", floor, ceiling)

        if channel is None or channel not in [0, 1]:
            raise Exception("Channel should be 1 or 0. Currently:", channel)
        
        self.floor = floor
        self.ceiling = ceiling
        
        self.mcp = MCP3208(channel=self.channel,differential=False,max_voltage=3.3,clock_pin=11,mosi_pin=10,miso_pin=9,select_pin=8)

    def __get_raw_value(self):
        return self.mcp.value

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
