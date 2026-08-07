from typing import List, TypedDict

from ..utils.pid_tuning import PidType


class CoordinateMqtt(TypedDict):
    lat: float
    lon: float


class FloatMsgMqtt(TypedDict):
    data: float


class BoolMsgMqtt(TypedDict):
    data: bool


class StringMqtt(TypedDict):
    data: str


class DepthControlCmdMqtt(TypedDict):
    depth_target: float
    pitch_target: float
    depth_pid_type: PidType
    pitch_pid_type: PidType


class AssignmentMqtt(TypedDict):
    coordinate: CoordinateMqtt
    target_depth: float
    sync_after: bool


class NavigationMissionMqtt(TypedDict):
    assignments: List[AssignmentMqtt]


class SubmergedCoordinateMqtt(TypedDict):
    coordinate: CoordinateMqtt
    depth: float


class TracedRouteMqtt(TypedDict):
    path: List[SubmergedCoordinateMqtt]
    started_at: str
    ended_at: str
    duration_seconds: float
    xy_distance_covered_meters: float
    average_depth_meters: float
