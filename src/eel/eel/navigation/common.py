from eel_interfaces.msg import Coordinate
from typing import TypedDict
from ..utils.nav import (
    get_distance_in_meters,
    get_relative_bearing_in_degrees,
    get_next_rudder_turn,
)


class LatLon(TypedDict):
    lat: float
    lon: float


def get_2d_distance(pos1: LatLon, pos2: LatLon) -> float:
    return get_distance_in_meters(
        pos1["lat"],
        pos1["lon"],
        pos2["lat"],
        pos2["lon"],
    )


def get_2d_distance_from_coords(coord1: Coordinate, coord2: Coordinate) -> float:
    return get_distance_in_meters(
        coord1.lat,
        coord1.lon,
        coord2.lat,
        coord2.lon,
    )


def get_relative_bearing(pos1: LatLon, pos2: LatLon) -> float:
    return get_relative_bearing_in_degrees(
        pos1["lat"],
        pos1["lon"],
        pos2["lat"],
        pos2["lon"],
    )


TOLERANCE_IN_METERS = 5.0
