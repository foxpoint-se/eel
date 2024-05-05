from typing import TypedDict
from geopy import distance


class LatLon(TypedDict):
    lat: float
    lon: float


class Localizer:
    def __init__(
        self,
        start_lat: float = 0.0,
        start_lon: float = 0.0,
        start_time_sec: float = 0.0,
        # linear_velocity_mps: float = 1.0,
    ) -> None:
        self._start_lat: float = start_lat
        self._start_lon: float = start_lon
        self._start_time_sec: float = start_time_sec
        # self._linear_velocity_mps: float = linear_velocity_mps
        self._current_position: LatLon = LatLon(lat=start_lat, lon=start_lon)
        self._current_speed_mps: float = 0.0
        self._current_heading: float = 0.0
        self._meters_traveled: float = 0.0

    def get_current_position(self) -> LatLon:
        return self._current_position

    def update_speed_mps(self, new_speed_mps: float) -> None:
        self._current_speed_mps = new_speed_mps

    def update_heading(self, new_heading: float) -> None:
        self._current_heading = new_heading

    def update_with_new_fix(self, new_fix: LatLon) -> None:
        self._current_position = new_fix

    def calculate_position(self, current_time_sec: float) -> None:
        time_delta = current_time_sec - self._start_time_sec
        meters_traveled = self._current_speed_mps * time_delta
        self._meters_traveled = meters_traveled
        new_position = distance.distance(meters=meters_traveled).destination(
            (self._current_position["lat"], self._current_position["lon"]),
            bearing=self._current_heading,
        )
        self._current_position = LatLon(
            lat=new_position.latitude, lon=new_position.longitude
        )
