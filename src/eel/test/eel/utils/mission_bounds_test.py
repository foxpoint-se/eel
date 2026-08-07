from eel.utils.mission_bounds import are_mission_waypoints_valid, is_mission_waypoint_valid

MAX_DEPTH_M = 3.0


def test__when_mission_has_no_waypoints__should_accept() -> None:
    assert are_mission_waypoints_valid([])


def test__when_waypoint_is_within_limits__should_accept() -> None:
    assert is_mission_waypoint_valid(59.0, 18.0, 1.0, max_depth_m=MAX_DEPTH_M)


def test__when_coordinate_lat_is_invalid__should_reject() -> None:
    assert not is_mission_waypoint_valid(91.0, 18.0, 1.0, max_depth_m=MAX_DEPTH_M)


def test__when_coordinate_lon_is_invalid__should_reject() -> None:
    assert not is_mission_waypoint_valid(59.0, 181.0, 1.0, max_depth_m=MAX_DEPTH_M)


def test__when_target_depth_exceeds_max__should_reject() -> None:
    assert not is_mission_waypoint_valid(59.0, 18.0, 4.0, max_depth_m=MAX_DEPTH_M)


def test__when_target_depth_is_nan__should_reject() -> None:
    assert not is_mission_waypoint_valid(59.0, 18.0, float("nan"), max_depth_m=MAX_DEPTH_M)


def test__when_target_depth_is_infinite__should_reject() -> None:
    assert not is_mission_waypoint_valid(59.0, 18.0, float("inf"), max_depth_m=MAX_DEPTH_M)


def test__when_one_waypoint_is_invalid__should_reject_mission() -> None:
    waypoints = [(59.0, 18.0, 1.0), (59.0, 18.0, float("inf"))]
    assert not are_mission_waypoints_valid(waypoints, max_depth_m=MAX_DEPTH_M)
