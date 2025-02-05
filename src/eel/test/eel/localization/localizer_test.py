import math
import time
from eel.localization.localizer import Localizer, LatLon


def test__when_instantiated__should_not_have_a_position() -> None:
    instance_to_test = Localizer()

    assert instance_to_test.get_calculated_position(0) is None


def test__when_instantiated__should_have_speed_zero() -> None:
    instance_to_test = Localizer()
    assert instance_to_test._current_speed_mps == 0.0


def test__when_instantiated__should_have_heading_zero() -> None:
    instance_to_test = Localizer()
    assert instance_to_test._current_heading == 0.0


def test__when_new_known_position__should_return_that_one_as_current_pos() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    known_position = LatLon(lat=8.0, lon=9.0)

    instance_to_test.update_known_position(known_position)

    actual = instance_to_test.get_calculated_position(time.time())

    assert actual and math.isclose(
        a=actual["lat"], b=known_position["lat"], rel_tol=0.000001
    )
    assert math.isclose(a=actual["lon"], b=known_position["lon"], rel_tol=0.000001)


def test__when_speed_zero__should_not_change_position() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(0.0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    actual = instance_to_test.get_calculated_position(time.time())
    assert actual and actual["lat"] == 0.0
    assert actual["lon"] == 0.0


def test__when_speed_zero__should_not_travel_any_distance() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(0.0)
    instance_to_test.get_calculated_position(time.time() + 10.0)
    assert instance_to_test.get_total_meters_traveled() == 0.0


def test__when_speed_1_mps__should_travel_1_m_after_1_second() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.get_calculated_position(1)
    assert instance_to_test.get_total_meters_traveled() == 1.0


def test__when_speed_1_mps_and_heading_zero__should_keep_same_lon() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 20, "lon": 10})
    instance_to_test.update_speed_mps(1.0)
    actual = instance_to_test.get_calculated_position(100.0)
    assert actual and actual["lon"] == 10


def test__when_speed_1_mps_and_heading_zero__lat_should_be_slightly_bigger() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 10})
    instance_to_test.update_speed_mps(1.0)
    actual = instance_to_test.get_calculated_position(1.0)
    assert actual and 9.043695e-06 > actual["lat"] > 9.043693e-060


def test__when_speed_1_mps_and_heading_90__should_keep_same_lat() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_known_position({"lat": 0, "lon": 10})
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(90)
    actual = instance_to_test.get_calculated_position(time.time() + 1.0)
    assert actual and actual["lat"] == 0.0


def test__when_speed_1_mps_and_heading_45__lat_and_lon_should_be_slightly_bigger() -> (
    None
):
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.get_calculated_position(0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(45)

    actual = instance_to_test.get_calculated_position(1.0)
    assert actual and 6.3948579e-06 > actual["lat"] > 6.3948578e-06
    assert 6.3520483e-06 > actual["lon"] > 6.3520482e-06


def test__when_speed_1mps_and_heading_45_and_100s_travel__should_move_a_fair_bit() -> (
    None
):
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(45)
    actual = instance_to_test.get_calculated_position(100.0)
    assert actual and 0.000639486 > actual["lat"] > 0.00063947
    assert 0.0006352049 > actual["lon"] > 0.0006352047


def test__when_one_known_and_sequence_of_moves__should_move_as_specified() -> None:
    current_time = time.time()
    instance_to_test = Localizer(start_time_sec=current_time)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_speed_mps(1.0)
    # move north 10 seconds
    current_time += 10.0
    instance_to_test.update_heading(0)
    instance_to_test.get_calculated_position(current_time_sec=current_time)

    # move east 10 seconds
    current_time += 10
    instance_to_test.update_heading(90)
    instance_to_test.get_calculated_position(current_time_sec=current_time)

    # move south 40 seconds
    current_time += 40
    instance_to_test.update_heading(180)
    actual = instance_to_test.get_calculated_position(current_time_sec=current_time)

    assert instance_to_test.get_total_meters_traveled() == 60

    assert actual and math.isclose(a=actual["lat"], b=-0.000271311, rel_tol=0.00001)
    assert math.isclose(a=actual["lon"], b=8.98315284e-05, rel_tol=0.00001)


def test__when_moving_from_two_very_distant_knowns__distance_traveled_should_not_be_huge() -> (
    None
):
    instance_to_test = Localizer()
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(0)
    instance_to_test.get_calculated_position(10)

    # known position is very far from previous
    instance_to_test.update_known_position({"lat": 20, "lon": 20})
    instance_to_test.get_calculated_position(time.time() + 20.0)

    actual = round(instance_to_test.get_total_meters_traveled(), 1)
    assert math.isclose(a=actual, b=20, rel_tol=0.01)


def positions_are_close(pos_1: LatLon, pos_2: LatLon) -> bool:
    return math.isclose(
        a=pos_1["lat"], b=pos_2["lat"], rel_tol=0.00001
    ) and math.isclose(a=pos_1["lon"], b=pos_2["lon"], rel_tol=0.00001)


def test__when_depth_is_three_decimeters__should_discard_gps_position() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(0)

    position_before_dive = instance_to_test.get_calculated_position(time.time())

    instance_to_test.update_depth(0.3)
    wonky_position_under_water: LatLon = {"lat": 1, "lon": 1}
    instance_to_test.update_known_position(wonky_position_under_water)
    position_after_dive = instance_to_test.get_calculated_position(time.time())

    assert position_after_dive and not positions_are_close(
        wonky_position_under_water, position_after_dive
    ), "positions are too close, which indicates that gps position was used during dive"

    assert position_before_dive and positions_are_close(
        position_before_dive, position_after_dive
    ), "position should not have changed, since gps should be discarded during dive"


def test__when_speed_zero_and_drift_one_mps_should_give_new_position() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_drift_speed_mps(1.0)
    instance_to_test.update_drift_bearing(90.0)  # Should be east bound
    position_after_one_sec = instance_to_test.get_calculated_position(1.0)

    assert position_after_one_sec and not positions_are_close(
        {"lat": 0, "lon": 0}, position_after_one_sec
    )


def test__when_speed_zero_and_drift_one_mps_should_drift_one_meter_after_one_second() -> (
    None
):
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    instance_to_test.update_drift_speed_mps(1.0)
    instance_to_test.update_drift_bearing(90.0)  # Should be east bound
    position_after_one_sec = instance_to_test.get_calculated_position(1.0)

    assert position_after_one_sec and positions_are_close(
        {"lat": 0.0, "lon": 8.9831528e-06}, position_after_one_sec
    )


def test__when_speed_zero_and_drift_zero_mps_should_not_drift() -> None:
    instance_to_test = Localizer(start_time_sec=0)
    instance_to_test.update_known_position({"lat": 0, "lon": 0})
    position_after_one_sec = instance_to_test.get_calculated_position(1.0)

    assert position_after_one_sec and positions_are_close(
        {"lat": 0.0, "lon": 0.0}, position_after_one_sec
    )
