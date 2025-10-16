from typing import Literal, TypedDict
from ..utils.topics import (
    FRONT_TANK_CMD,
    FRONT_TANK_STATUS,
    REAR_TANK_CMD,
    REAR_TANK_STATUS,
)

# Commands we used to run
# Maxxed
# ros2 run eel tank --ros-args -p cmd_topic:=tank_front/cmd -p status_topic:=tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p distance_sensor_channel:=0 -p tank_floor_value:=0.647 -p tank_ceiling_value:=0.18
# ros2 run eel tank --ros-args -p cmd_topic:=tank_rear/cmd -p status_topic:=tank_rear/status -p motor_pin:=24 -p direction_pin:=25 -p distance_sensor_channel:=1 -p tank_floor_value:=0.71 -p tank_ceiling_value:=0.29

# Conservative
# ros2 run eel tank --ros-args -p cmd_topic:=tank_front/cmd -p status_topic:=tank_front/status -p motor_pin:=23 -p direction_pin:=18 -p distance_sensor_channel:=0 -p tank_floor_value:=0.55 -p tank_ceiling_value:=0.3 -p simulate:=true
# ros2 run eel tank --ros-args -p cmd_topic:=tank_rear/cmd -p status_topic:=tank_rear/status -p motor_pin:=24 -p direction_pin:=25 -p distance_sensor_channel:=1 -p tank_floor_value:=0.6 -p tank_ceiling_value:=0.4 -p simulate:=true

FRONT_MOTOR_PIN = 23
FRONT_DIRECTION_PIN = 18
FRONT_DISTANCE_SENSOR_CHANNEL = 0

REAR_MOTOR_PIN = 24
REAR_DIRECTION_PIN = 25
REAR_DISTANCE_SENSOR_CHANNEL = 1


class TankRange(TypedDict):
    floor: float
    ceiling: float


FRONT_RANGE_MAX: TankRange = {"floor": 0.647, "ceiling": 0.18}

REAR_RANGE_MAX: TankRange = {"floor": 0.71, "ceiling": 0.29}

FRONT_RANGE_CONSERVATIVE: TankRange = {"floor": 0.55, "ceiling": 0.3}

REAR_RANGE_CONSERVATIVE: TankRange = {"floor": 0.55, "ceiling": 0.3}


class TankConfig(TypedDict):
    cmd_topic: str
    status_topic: str
    motor_pin: int
    direction_pin: int
    distance_sensor_channel: int
    tank_floor_value: float
    tank_ceiling_value: float


front_config: TankConfig = {
    "cmd_topic": FRONT_TANK_CMD,
    "status_topic": FRONT_TANK_STATUS,
    "motor_pin": FRONT_MOTOR_PIN,
    "direction_pin": FRONT_DIRECTION_PIN,
    "distance_sensor_channel": FRONT_DISTANCE_SENSOR_CHANNEL,
    "tank_floor_value": FRONT_RANGE_CONSERVATIVE["floor"],
    "tank_ceiling_value": FRONT_RANGE_CONSERVATIVE["ceiling"],
}

rear_config: TankConfig = {
    "cmd_topic": REAR_TANK_CMD,
    "status_topic": REAR_TANK_STATUS,
    "motor_pin": REAR_MOTOR_PIN,
    "direction_pin": REAR_DIRECTION_PIN,
    "distance_sensor_channel": REAR_DISTANCE_SENSOR_CHANNEL,
    "tank_floor_value": REAR_RANGE_CONSERVATIVE["floor"],
    "tank_ceiling_value": REAR_RANGE_CONSERVATIVE["ceiling"],
}


def get_tank_config(tank_type: str) -> TankConfig:
    if tank_type == "front":
        return front_config
    elif tank_type == "rear":
        return rear_config

    raise ValueError(f"Could not find matching config for {tank_type=}")
