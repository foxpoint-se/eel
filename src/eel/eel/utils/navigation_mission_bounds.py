from eel_interfaces.msg import NavigationAssignment, NavigationMission

from .actuator_bounds import MAX_DEPTH_M
from .mission_bounds import is_mission_waypoint_valid


def is_mission_assignment_valid(assignment: NavigationAssignment, max_depth_m: float = MAX_DEPTH_M) -> bool:
    coord = assignment.coordinate
    return is_mission_waypoint_valid(coord.lat, coord.lon, assignment.target_depth, max_depth_m)


def is_navigation_mission_valid(msg: NavigationMission, max_depth_m: float = MAX_DEPTH_M) -> bool:
    if len(msg.assignments) == 0:
        return True
    return all(is_mission_assignment_valid(assignment, max_depth_m) for assignment in msg.assignments)
