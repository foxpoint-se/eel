#!/usr/bin/python3
from .hardware import HardwareSensor

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

    sensor = HardwareSensor(floor, ceiling, channel)

    while True:
        level = sensor.get_level()
        raw_val = sensor._get_raw_value()
        print(f"{level=}")
        print(f"{raw_val=}")
        time.sleep(1)
