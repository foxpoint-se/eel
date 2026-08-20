"""Local ENU meters around a fixed WGS84 origin (Gröndal sim area)."""

from __future__ import annotations

import math

# Fixed Gröndal origin for plant meters→lat/lon (was shared with old gnss_sim).
MAP_ORIGIN_LAT = 59.309395
MAP_ORIGIN_LON = 17.974279

_METERS_PER_DEG_LAT = 111_320.0


def meters_to_latlon(
    east_m: float,
    north_m: float,
    origin_lat: float = MAP_ORIGIN_LAT,
    origin_lon: float = MAP_ORIGIN_LON,
) -> tuple[float, float]:
    """Convert east/north meters from origin to WGS84 lat/lon."""
    lat = origin_lat + north_m / _METERS_PER_DEG_LAT
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    lon = origin_lon + east_m / meters_per_deg_lon
    return lat, lon


def latlon_to_meters(
    lat: float,
    lon: float,
    origin_lat: float = MAP_ORIGIN_LAT,
    origin_lon: float = MAP_ORIGIN_LON,
) -> tuple[float, float]:
    """Convert WGS84 lat/lon to east/north meters from origin."""
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    east_m = (lon - origin_lon) * meters_per_deg_lon
    north_m = (lat - origin_lat) * _METERS_PER_DEG_LAT
    return east_m, north_m


def format_latlon(lat: float, lon: float) -> str:
    return f"{lat:.6f}, {lon:.6f}"
