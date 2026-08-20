"""Shared front/rear tank bringup config (boat hardware). No simulate.

Active floor/ceiling are the values used by most boat launches + CI.

Other values seen in the repo (not proven on the boat — re-calibrate later):
  front 0.647 / 0.18  and rear 0.71 / 0.29  — old single-tank launch
  front 0.55 / 0.3    and rear 0.6 / 0.4    — notes called these "conservative"
"""

from typing import TypedDict


class TankHardwareConfig(TypedDict):
    cmd_topic: str
    status_topic: str
    motor_pin: int
    direction_pin: int
    distance_sensor_channel: int
    tank_floor_value: float
    tank_ceiling_value: float


FRONT_TANK: TankHardwareConfig = {
    "cmd_topic": "tank_front/cmd",
    "status_topic": "tank_front/status",
    "motor_pin": 23,
    "direction_pin": 18,
    "distance_sensor_channel": 0,
    "tank_floor_value": 0.66,
    "tank_ceiling_value": 0.16,
}

REAR_TANK: TankHardwareConfig = {
    "cmd_topic": "tank_rear/cmd",
    "status_topic": "tank_rear/status",
    "motor_pin": 24,
    "direction_pin": 25,
    "distance_sensor_channel": 1,
    "tank_floor_value": 0.325,
    "tank_ceiling_value": 0.005,
}


def tank_config(tank: str) -> TankHardwareConfig:
    if tank == "front":
        return FRONT_TANK
    if tank == "rear":
        return REAR_TANK
    raise ValueError(f"tank must be 'front' or 'rear', got {tank!r}")
