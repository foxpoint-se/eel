from dataclasses import dataclass
from typing import Literal, TypedDict

from geopy import distance

GnssTrustState = Literal["trusting", "dead_reckoning", "reacquiring"]


class LatLon(TypedDict):
    lat: float
    lon: float


@dataclass(frozen=True)
class GnssBlackoutConfig:
    """Depth-gated GNSS trust with hysteresis and fix clustering on re-entry."""

    enter_depth_m: float
    exit_depth_m: float
    cluster_radius_m: float
    fixes_required: int
    max_outlier_distance_m: float
    reacquire_timeout_sec: float


DEFAULT_GNSS_BLACKOUT_CONFIG = GnssBlackoutConfig(
    enter_depth_m=0.35,
    exit_depth_m=0.15,
    cluster_radius_m=5.0,
    fixes_required=2,
    max_outlier_distance_m=1000.0,
    reacquire_timeout_sec=30.0,
)


def require_valid_gnss_blackout_config(config: GnssBlackoutConfig) -> GnssBlackoutConfig:
    if config.enter_depth_m < 0:
        raise ValueError(f"enter_depth_m must be >= 0, got {config.enter_depth_m}")
    if config.exit_depth_m < 0:
        raise ValueError(f"exit_depth_m must be >= 0, got {config.exit_depth_m}")
    if config.exit_depth_m > config.enter_depth_m:
        raise ValueError(f"exit_depth_m ({config.exit_depth_m}) must be <= enter_depth_m ({config.enter_depth_m})")
    if config.cluster_radius_m <= 0:
        raise ValueError(f"cluster_radius_m must be > 0, got {config.cluster_radius_m}")
    if config.fixes_required < 1:
        raise ValueError(f"fixes_required must be >= 1, got {config.fixes_required}")
    if config.max_outlier_distance_m <= 0:
        raise ValueError(f"max_outlier_distance_m must be > 0, got {config.max_outlier_distance_m}")
    if config.reacquire_timeout_sec < 0:
        raise ValueError(f"reacquire_timeout_sec must be >= 0, got {config.reacquire_timeout_sec}")
    return config


def distance_m_between(a: LatLon, b: LatLon) -> float:
    return float(
        distance.distance((a["lat"], a["lon"]), (b["lat"], b["lon"])).meters,
    )


def trust_state_after_depth_change(
    state: GnssTrustState,
    depth_m: float,
    config: GnssBlackoutConfig = DEFAULT_GNSS_BLACKOUT_CONFIG,
) -> GnssTrustState:
    if state == "trusting" and depth_m >= config.enter_depth_m:
        return "dead_reckoning"
    if state == "dead_reckoning" and depth_m <= config.exit_depth_m:
        return "reacquiring"
    if state == "reacquiring" and depth_m >= config.enter_depth_m:
        return "dead_reckoning"
    return state


def centroid(fixes: list[LatLon]) -> LatLon:
    return {
        "lat": sum(fix["lat"] for fix in fixes) / len(fixes),
        "lon": sum(fix["lon"] for fix in fixes) / len(fixes),
    }


def clustered_fixes(
    pending_fixes: list[LatLon],
    config: GnssBlackoutConfig = DEFAULT_GNSS_BLACKOUT_CONFIG,
) -> LatLon | None:
    if config.fixes_required < 1:
        return None
    if len(pending_fixes) < config.fixes_required:
        return None
    recent = pending_fixes[-config.fixes_required :]
    center = centroid(recent)
    if all(distance_m_between(center, fix) <= config.cluster_radius_m for fix in recent):
        return center
    return None


def is_obviously_invalid_fix(
    fix: LatLon,
    reference: LatLon,
    config: GnssBlackoutConfig = DEFAULT_GNSS_BLACKOUT_CONFIG,
) -> bool:
    return distance_m_between(fix, reference) > config.max_outlier_distance_m


def reacquire_fallback_fix(
    pending_fixes: list[LatLon],
    reacquiring_for_sec: float,
    config: GnssBlackoutConfig = DEFAULT_GNSS_BLACKOUT_CONFIG,
) -> LatLon | None:
    if not pending_fixes:
        return None
    if reacquiring_for_sec < config.reacquire_timeout_sec:
        return None
    return pending_fixes[-1]
