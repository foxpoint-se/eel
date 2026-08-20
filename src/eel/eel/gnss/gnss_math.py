"""Valid lat/lon gate shared by GNSS app path (no ROS)."""


def is_valid_gnss_latlon(lat: float, lon: float) -> bool:
    """Reject near-zero / unset fixes (same rule as the old gnss node)."""
    return abs(lat) > 0.1 and abs(lon) > 0.1
