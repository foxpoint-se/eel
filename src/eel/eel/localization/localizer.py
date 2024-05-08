import time
from typing import Optional, TypedDict
from geopy import distance


class LatLon(TypedDict):
    lat: float
    lon: float


class Localizer:
    def __init__(
        self,
        start_time_sec: float = time.time(),
    ) -> None:
        self._current_position: Optional[LatLon] = None
        self._last_recorded_at = start_time_sec
        self._current_speed_mps: float = 0.0
        self._current_heading: float = 0.0
        self._total_meters_traveled: float = 0.0

    def update_speed_mps(self, new_speed_mps: float) -> None:
        self._current_speed_mps = new_speed_mps

    def update_heading(self, new_heading: float) -> None:
        self._current_heading = new_heading

    def update_known_position(self, new_position: LatLon) -> None:
        self._current_position = new_position

    def get_total_meters_traveled(self) -> float:
        return self._total_meters_traveled

    def get_calculated_position(
        self, current_time_sec: float = time.time()
    ) -> Optional[LatLon]:
        if self._current_position:

            time_delta = current_time_sec - self._last_recorded_at

            meters_traveled = self._current_speed_mps * time_delta
            self._total_meters_traveled += meters_traveled
            new_position = distance.distance(meters=meters_traveled).destination(
                (self._current_position["lat"], self._current_position["lon"]),
                bearing=self._current_heading,
            )
            self._current_position = LatLon(
                lat=new_position.latitude, lon=new_position.longitude
            )

        self._last_recorded_at = current_time_sec

        return self._current_position
