import math
from enum import Enum


class NavigateGoalDecision(Enum):
    ACCEPT = "accept"
    REJECT_BUSY = "reject_busy"
    REJECT_NO_POSE = "reject_no_pose"
    REJECT_INVALID_COORDINATES = "reject_invalid_coordinates"
    REJECT_TOO_FAR = "reject_too_far"


def evaluate_navigate_goal(
    *,
    goal_in_progress: bool,
    has_pose: bool,
    coordinates_valid: bool,
    distance_to_target_m: float,
    distance_limit_m: float,
) -> NavigateGoalDecision:
    if goal_in_progress:
        return NavigateGoalDecision.REJECT_BUSY
    if not has_pose:
        return NavigateGoalDecision.REJECT_NO_POSE
    if not coordinates_valid:
        return NavigateGoalDecision.REJECT_INVALID_COORDINATES
    if not math.isfinite(distance_to_target_m) or distance_to_target_m >= distance_limit_m:
        return NavigateGoalDecision.REJECT_TOO_FAR
    return NavigateGoalDecision.ACCEPT
