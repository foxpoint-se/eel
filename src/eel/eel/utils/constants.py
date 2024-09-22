from enum import Enum

SIMULATE_PARAM = "simulate"
MOTOR_PIN_PARAM = "motor_pin"
DIRECTION_PIN_PARAM = "direction_pin"
DISTANCE_SENSOR_CHANNEL_PARAM = "distance_sensor_channel"
CMD_TOPIC_PARAM = "cmd_topic"
STATUS_TOPIC_PARAM = "status_topic"
TANK_FLOOR_VALUE_PARAM = "tank_floor_value"
TANK_CEILING_VALUE_PARAM = "tank_ceiling_value"


class NavigationMissionStatus(Enum):
    WAITING_FOR_MISSION = 0
    MISSION_AQUIRED = 1
    MISSION_STARTED = 2
    MISSION_CANCELLED = 3
    MISSION_FINISHED = 4
