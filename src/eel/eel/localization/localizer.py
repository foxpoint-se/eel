import time
from typing import Optional

from geopy import distance

from ..motion.planar_motion import motor_to_speed_mps, planar_delta_from_speed_mps, planar_delta_m
from .gnss_blackout import (
    DEFAULT_GNSS_BLACKOUT_CONFIG,
    GnssBlackoutConfig,
    GnssTrustState,
    LatLon,
    clustered_fixes,
    is_obviously_invalid_fix,
    reacquire_fallback_fix,
    trust_state_after_depth_change,
)

__all__ = ["LatLon", "Localizer"]


def _position_after_enu_delta(lat: float, lon: float, east_m: float, north_m: float) -> LatLon:
    if north_m != 0.0:
        point = distance.distance(meters=abs(north_m)).destination(
            (lat, lon),
            bearing=0.0 if north_m > 0.0 else 180.0,
        )
        lat, lon = point.latitude, point.longitude
    if east_m != 0.0:
        point = distance.distance(meters=abs(east_m)).destination(
            (lat, lon),
            bearing=90.0 if east_m > 0.0 else 270.0,
        )
        lat, lon = point.latitude, point.longitude
    return {"lat": lat, "lon": lon}


class Localizer:
    def __init__(
        self,
        start_time_sec: float | None = None,
        gnss_config: GnssBlackoutConfig = DEFAULT_GNSS_BLACKOUT_CONFIG,
    ) -> None:
        self._gnss_config = gnss_config
        self._current_position: Optional[LatLon] = None
        self._last_recorded_at = time.time() if start_time_sec is None else start_time_sec
        self._motor_cmd: float = 0.0
        self._pitch_deg: float = 0.0
        self._current_heading: float = 0.0
        self._total_meters_traveled: float = 0.0
        self._current_depth: float = 0.0
        self._drift_speed: float = 0.0
        self._drift_bearing: float = 0.0
        self._gnss_trust_state: GnssTrustState = "trusting"
        self._pending_gnss_fixes: list[LatLon] = []
        self._reacquiring_since_sec: Optional[float] = None

    def update_motor_cmd(self, motor_cmd: float) -> None:
        self._motor_cmd = motor_cmd

    def update_pitch_deg(self, pitch_deg: float) -> None:
        self._pitch_deg = pitch_deg

    def update_heading(self, new_heading: float) -> None:
        self._current_heading = new_heading

    def update_drift_speed_mps(self, new_drift_speed: float) -> None:
        self._drift_speed = new_drift_speed

    def update_drift_bearing(self, new_drift_bearing: float) -> None:
        self._drift_bearing = new_drift_bearing

    def update_known_position(self, new_position: LatLon, now_sec: float | None = None) -> None:
        now = now_sec if now_sec is not None else time.time()
        if self._gnss_trust_state == "trusting":
            self._current_position = new_position
            self._advance_last_recorded_at(now)
            return

        if self._gnss_trust_state == "dead_reckoning":
            return

        reference = self._current_position
        if reference is not None and is_obviously_invalid_fix(new_position, reference, self._gnss_config):
            return

        self._pending_gnss_fixes.append(new_position)
        accepted = clustered_fixes(self._pending_gnss_fixes, self._gnss_config)
        if accepted is None:
            if self._reacquiring_since_sec is None:
                reacquiring_for = 0.0
            else:
                reacquiring_for = now - self._reacquiring_since_sec
            accepted = reacquire_fallback_fix(self._pending_gnss_fixes, reacquiring_for, self._gnss_config)

        if accepted is not None:
            self._accept_gnss_fix(accepted, now)

    def get_total_meters_traveled(self) -> float:
        return self._total_meters_traveled

    def update_depth(self, new_depth: float, now_sec: float | None = None) -> None:
        previous_state = self._gnss_trust_state
        self._current_depth = new_depth
        self._gnss_trust_state = trust_state_after_depth_change(
            self._gnss_trust_state,
            new_depth,
            self._gnss_config,
        )
        now = now_sec if now_sec is not None else time.time()
        if self._gnss_trust_state != previous_state and self._gnss_trust_state == "reacquiring":
            self._pending_gnss_fixes = []
            self._reacquiring_since_sec = now
        if self._gnss_trust_state != previous_state and self._gnss_trust_state == "dead_reckoning":
            self._pending_gnss_fixes = []
            self._reacquiring_since_sec = None

    def get_calculated_position(
        self,
        current_time_sec: float,
    ) -> Optional[LatLon]:
        if self._current_position:
            time_delta = current_time_sec - self._last_recorded_at
            if time_delta > 0:
                motor_east_m, motor_north_m = planar_delta_m(
                    self._motor_cmd,
                    self._current_heading,
                    time_delta,
                    pitch_deg=self._pitch_deg,
                )
                drift_east_m, drift_north_m = planar_delta_from_speed_mps(
                    self._drift_speed,
                    self._drift_bearing,
                    time_delta,
                )
                speed_mps = motor_to_speed_mps(self._motor_cmd, pitch_deg=self._pitch_deg)
                self._total_meters_traveled += speed_mps * time_delta
                self._current_position = _position_after_enu_delta(
                    self._current_position["lat"],
                    self._current_position["lon"],
                    motor_east_m + drift_east_m,
                    motor_north_m + drift_north_m,
                )

        self._advance_last_recorded_at(current_time_sec)

        return self._current_position

    def _accept_gnss_fix(self, position: LatLon, now_sec: float) -> None:
        self._current_position = position
        self._advance_last_recorded_at(now_sec)
        self._pending_gnss_fixes = []
        self._reacquiring_since_sec = None
        self._gnss_trust_state = "trusting"

    def _advance_last_recorded_at(self, now_sec: float) -> None:
        self._last_recorded_at = max(self._last_recorded_at, now_sec)
