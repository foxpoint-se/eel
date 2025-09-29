#!/usr/bin/python3

from gpiozero import MCP3208


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


class RealDistanceSensor:
    def __init__(self, floor: float, ceiling: float, channel: int) -> None:
        super().__init__()
        self.channel = channel
        self.floor = floor
        self.ceiling = ceiling

        self.mcp = MCP3208(
            channel=self.channel,
            differential=False,
            max_voltage=3.3,
            clock_pin=11,
            mosi_pin=10,
            miso_pin=9,
            select_pin=8,
        )

    def __get_raw_value(self) -> float:
        # print("raw", self.mcp.value)
        return self.mcp.value

    def __get_pretty_value(self) -> float:
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

# "distance_sensor_channel": 1,
# "tank_floor_value": 0.325,
# "tank_ceiling_value": 0.005,
if __name__ == "__main__":
    import time
    
    # rear
    floor = 0.71
    ceiling = 0.29
    channel = 1
    
    # front
    # floor = 0.647
    # ceiling = 0.18
    # channel = 0

    rear_sensor = RealDistanceSensor(floor, ceiling, channel)
    
    while True:
        val = rear_sensor.get_level()
        time.sleep(1)
