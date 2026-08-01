import math


def is_dive_goal_valid(
    *,
    wanted_depth: float,
    dive_time: float,
    max_depth_m: float,
    max_dive_time_sec: float,
) -> bool:
    if not math.isfinite(wanted_depth) or not math.isfinite(dive_time):
        return False
    if wanted_depth < 0 or dive_time <= 0:
        return False
    if wanted_depth > max_depth_m:
        return False
    if dive_time > max_dive_time_sec:
        return False
    return True
