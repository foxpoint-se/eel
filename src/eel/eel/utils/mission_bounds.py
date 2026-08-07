from .actuator_bounds import MAX_DEPTH_M, is_valid_coordinate, is_valid_depth_target


def is_mission_waypoint_valid(
    lat: float,
    lon: float,
    target_depth: float,
    max_depth_m: float = MAX_DEPTH_M,
) -> bool:
    if not is_valid_coordinate(lat, lon):
        return False
    return is_valid_depth_target(target_depth, max_depth_m)


def are_mission_waypoints_valid(
    waypoints: list[tuple[float, float, float]],
    max_depth_m: float = MAX_DEPTH_M,
) -> bool:
    if len(waypoints) == 0:
        return True
    return all(is_mission_waypoint_valid(lat, lon, depth, max_depth_m) for lat, lon, depth in waypoints)
