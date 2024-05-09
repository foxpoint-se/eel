#!/usr/bin/python3

import busio
import digitalio
import board
import time

from adafruit_mcp3xxx.analog_in import AnalogIn
from gpiozero import MCP3208

# channel 1 is rear and 0 is front
# Rear motor empty limit: 0.28726651202539366
# Rear motor fill limit: 0.05945549993895738

channel = 1
mcp = MCP3208(channel=channel, differential=False, max_voltage=3.3)

# front_floor = 0.66
# front_ceiling = 0.16

rear_floor = 0.325
rear_ceiling = 0.005


def translate_from_range_to_range(value, from_min, from_max, to_min, to_max):
    # Figure out how 'wide' each range is
    left_span = from_max - from_min
    right_span = to_max - to_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - from_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return to_min + (value_scaled * right_span)


print(f"TEST SCRIPT INIT {channel=} {mcp=}")


while True:
    print(
        f"Raw value{mcp.value} in percentage {translate_from_range_to_range(mcp.value, rear_floor, rear_ceiling, 0.0, 1.0)}"
    )
    time.sleep(1)
