from eel_interfaces.msg import (
    Coordinate,
    TracedRoute,
    SubmergedCoordinate,
)
from typing import List, TypedDict


class CoordinateMqtt(TypedDict):
    lat: float
    lon: float


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


def transform_coordinate_msg(msg: Coordinate) -> CoordinateMqtt:
    return {
        "lat": msg.lat,
        "lon": msg.lon,
    }


def to_submerged_coord_mqtt(msg: SubmergedCoordinate) -> SubmergedCoordinateMqtt:
    return {
        "coordinate": transform_coordinate_msg(msg.coordinate),
        "depth": msg.depth,
    }


def to_traced_route_mqtt(msg: TracedRoute) -> TracedRouteMqtt:
    return {
        "path": [to_submerged_coord_mqtt(c) for c in msg.path],
        "average_depth_meters": msg.average_depth_meters,
        "duration_seconds": msg.duration_seconds,
        "ended_at": msg.ended_at,
        "started_at": msg.started_at,
        "xy_distance_covered_meters": msg.xy_distance_covered_meters,
    }
