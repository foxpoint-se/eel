from eel.navigation.navigation_goal_admission import NavigateGoalDecision, evaluate_navigate_goal

DISTANCE_LIMIT_M = 2500.0


def test__when_goal_in_progress__should_reject_busy() -> None:
    assert (
        evaluate_navigate_goal(
            goal_in_progress=True,
            has_pose=True,
            coordinates_valid=True,
            distance_to_target_m=100.0,
            distance_limit_m=DISTANCE_LIMIT_M,
        )
        == NavigateGoalDecision.REJECT_BUSY
    )


def test__when_pose_missing__should_reject_no_pose() -> None:
    assert (
        evaluate_navigate_goal(
            goal_in_progress=False,
            has_pose=False,
            coordinates_valid=False,
            distance_to_target_m=100.0,
            distance_limit_m=DISTANCE_LIMIT_M,
        )
        == NavigateGoalDecision.REJECT_NO_POSE
    )


def test__when_coordinates_invalid__should_reject() -> None:
    assert (
        evaluate_navigate_goal(
            goal_in_progress=False,
            has_pose=True,
            coordinates_valid=False,
            distance_to_target_m=100.0,
            distance_limit_m=DISTANCE_LIMIT_M,
        )
        == NavigateGoalDecision.REJECT_INVALID_COORDINATES
    )


def test__when_target_too_far__should_reject_distance() -> None:
    assert (
        evaluate_navigate_goal(
            goal_in_progress=False,
            has_pose=True,
            coordinates_valid=True,
            distance_to_target_m=DISTANCE_LIMIT_M,
            distance_limit_m=DISTANCE_LIMIT_M,
        )
        == NavigateGoalDecision.REJECT_TOO_FAR
    )


def test__when_pose_ok_and_target_near__should_accept() -> None:
    assert (
        evaluate_navigate_goal(
            goal_in_progress=False,
            has_pose=True,
            coordinates_valid=True,
            distance_to_target_m=100.0,
            distance_limit_m=DISTANCE_LIMIT_M,
        )
        == NavigateGoalDecision.ACCEPT
    )


def test__when_distance_is_nan__should_reject() -> None:
    assert (
        evaluate_navigate_goal(
            goal_in_progress=False,
            has_pose=True,
            coordinates_valid=True,
            distance_to_target_m=float("nan"),
            distance_limit_m=DISTANCE_LIMIT_M,
        )
        == NavigateGoalDecision.REJECT_TOO_FAR
    )
