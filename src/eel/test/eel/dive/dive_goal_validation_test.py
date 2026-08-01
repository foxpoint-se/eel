from eel.dive.dive_goal_validation import is_dive_goal_valid

MAX_DEPTH_M = 5.0
MAX_DIVE_TIME_SEC = 120.0


def test__when_request_dive_time_exceeds_max__should_reject() -> None:
    assert not is_dive_goal_valid(
        wanted_depth=2.0,
        dive_time=121.0,
        max_depth_m=MAX_DEPTH_M,
        max_dive_time_sec=MAX_DIVE_TIME_SEC,
    )


def test__when_request_depth_exceeds_max__should_reject() -> None:
    assert not is_dive_goal_valid(
        wanted_depth=6.0,
        dive_time=10.0,
        max_depth_m=MAX_DEPTH_M,
        max_dive_time_sec=MAX_DIVE_TIME_SEC,
    )


def test__when_request_within_limits__should_accept() -> None:
    assert is_dive_goal_valid(
        wanted_depth=2.0,
        dive_time=10.0,
        max_depth_m=MAX_DEPTH_M,
        max_dive_time_sec=MAX_DIVE_TIME_SEC,
    )


def test__when_request_dive_time_is_negative__should_reject() -> None:
    assert not is_dive_goal_valid(
        wanted_depth=2.0,
        dive_time=-1.0,
        max_depth_m=MAX_DEPTH_M,
        max_dive_time_sec=MAX_DIVE_TIME_SEC,
    )


def test__when_request_depth_is_nan__should_reject() -> None:
    assert not is_dive_goal_valid(
        wanted_depth=float("nan"),
        dive_time=10.0,
        max_depth_m=MAX_DEPTH_M,
        max_dive_time_sec=MAX_DIVE_TIME_SEC,
    )
