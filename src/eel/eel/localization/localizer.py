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
        self._current_depth: float = 0.0
        self._drift_speed: float = 0.0
        self._drift_bearing: float = 0.0

    def update_speed_mps(self, new_speed_mps: float) -> None:
        self._current_speed_mps = new_speed_mps

    def update_heading(self, new_heading: float) -> None:
        self._current_heading = new_heading

    def update_drift_speed_mps(self, new_drift_speed) -> None:
        self._drift_speed = new_drift_speed

    def update_drift_bearing(self, new_drift_bearing) -> None:
        self._drift_bearing = new_drift_bearing

    def update_known_position(self, new_position: LatLon) -> None:
        if self._current_depth >= 0.3:
            return
        self._current_position = new_position

    def get_total_meters_traveled(self) -> float:
        return self._total_meters_traveled

    def update_depth(self, new_depth: float) -> None:
        self._current_depth = new_depth

    def get_calculated_position(
        self,
        current_time_sec: float,
    ) -> Optional[LatLon]:
        if self._current_position:

            time_delta = current_time_sec - self._last_recorded_at

            drift_meters = self._drift_speed * time_delta

            meters_traveled = self._current_speed_mps * time_delta
            self._total_meters_traveled += meters_traveled
            new_position = distance.distance(meters=meters_traveled).destination(
                (self._current_position["lat"], self._current_position["lon"]),
                bearing=self._current_heading,
            )

            final_position = distance.distance(meters=drift_meters).destination(
                (new_position.latitude, new_position.longitude), bearing=self._drift_bearing
            )

            self._current_position = LatLon(
                lat=final_position.latitude, lon=final_position.longitude
            )

        self._last_recorded_at = current_time_sec

        return self._current_position
