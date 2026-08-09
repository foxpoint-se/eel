"""Pure depth helpers for pressure app logic (no ROS)."""

import math


def calculate_center_depth(main_depth: float, pitch_deg: float, displacement: float = 0.375) -> float:
    pitch_rad = math.radians(pitch_deg)
    return main_depth - (displacement * math.sin(pitch_rad))


def get_depth_velocity(
    depth: float,
    previous_depth: float | None,
    now: float,
    previous_depth_at: float | None,
) -> float:
    if previous_depth is None or previous_depth_at is None:
        return 0.0
    time_delta = now - previous_depth_at
    if time_delta <= 0.0:
        return 0.0
    return (depth - previous_depth) / time_delta
