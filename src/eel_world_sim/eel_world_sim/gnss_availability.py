"""GNSS availability rules for the plant (no ROS)."""

# Match localization blackout enter depth for now.
GNSS_UNAVAILABLE_DEPTH_M = 0.35


def should_publish_gnss_fix(depth_m: float) -> bool:
    """Surface GPS only — silent at or below this depth (unavailable underwater)."""
    return depth_m < GNSS_UNAVAILABLE_DEPTH_M
