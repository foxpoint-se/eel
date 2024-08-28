from typing import List, TypedDict


class Coord3d(TypedDict):
    lat: float
    lon: float
    depth: float


class LatLon(TypedDict):
    lat: float
    lon: float


class TimedCoord3d(TypedDict):
    created_at: float
    coord: Coord3d


class Segment(TypedDict):
    finalized: bool
    started_at_seconds: float
    ended_at_seconds: float
    polyline: List[TimedCoord3d]
    accumulated_distance: float
