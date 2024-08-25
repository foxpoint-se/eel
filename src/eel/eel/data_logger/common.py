from typing import List, Literal, TypedDict


class Coord3d(TypedDict):
    x: float
    y: float
    z: float


class LatLon(TypedDict):
    lat: float
    lon: float


class TimedCoord3d(TypedDict):
    created_at: float
    coord: Coord3d


Segment = List[Coord3d]


class Segment2(TypedDict):
    finalized: bool
    started_at_seconds: float
    ended_at_seconds: float
    polyline: List[TimedCoord3d]
    accumulated_distance: float


class FinalizedSegment(TypedDict):
    started_at_seconds: float
    ended_at_seconds: float
    polyline: List[Coord3d]
    distance: float
    finalized: Literal[True]


class SegmentInProgress(TypedDict):
    # started_at_seconds: float
    # ended_at_seconds: float
    finalized: Literal[False]
    polyline: List[TimedCoord3d]
    accumulated_distance: float


def to_coord_3d(coord2d: LatLon, depth: float) -> Coord3d:
    return Coord3d(x=coord2d["lat"], y=coord2d["lon"], z=depth)
