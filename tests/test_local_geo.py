from math import isclose

from eel_world_sim.local_geo import MAP_ORIGIN_LAT, MAP_ORIGIN_LON, latlon_to_meters, meters_to_latlon


def test__when_at_origin__should_return_origin_latlon() -> None:
    lat, lon = meters_to_latlon(0.0, 0.0)
    assert isclose(lat, MAP_ORIGIN_LAT)
    assert isclose(lon, MAP_ORIGIN_LON)


def test__when_moved_north__should_increase_latitude() -> None:
    lat, lon = meters_to_latlon(0.0, 500.0)
    assert lat > MAP_ORIGIN_LAT
    assert isclose(lon, MAP_ORIGIN_LON)


def test__when_moved_east__should_increase_longitude() -> None:
    lat, lon = meters_to_latlon(500.0, 0.0)
    assert isclose(lat, MAP_ORIGIN_LAT)
    assert lon > MAP_ORIGIN_LON


def test__when_round_trip_meters__should_recover_east_north() -> None:
    east_m, north_m = 120.0, -80.0
    lat, lon = meters_to_latlon(east_m, north_m)
    back_east, back_north = latlon_to_meters(lat, lon)
    assert isclose(back_east, east_m, abs_tol=1e-6)
    assert isclose(back_north, north_m, abs_tol=1e-6)
