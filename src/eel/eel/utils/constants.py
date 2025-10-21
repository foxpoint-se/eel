from enum import Enum

SIMULATE_PARAM = "simulate"


class NavigationMissionStatus(Enum):
    WAITING_FOR_MISSION = 0
    MISSION_AQUIRED = 1
    MISSION_STARTED = 2
    MISSION_CANCELLED = 3
    MISSION_FINISHED = 4
