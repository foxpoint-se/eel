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


def get_relative_bearing(pos1: LatLon, pos2: LatLon) -> float:
    return get_relative_bearing_in_degrees(
        pos1["lat"],
        pos1["lon"],
        pos2["lat"],
        pos2["lon"],
    )
