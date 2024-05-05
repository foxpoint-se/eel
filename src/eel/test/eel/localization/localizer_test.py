from eel.localization.localizer import Localizer, LatLon


def test__when_instantiated__should_set_lat_lon_to_self() -> None:
    instance_to_test = Localizer(start_lat=1.111, start_lon=2.222)

    assert instance_to_test._start_lat == 1.111
    assert instance_to_test._start_lon == 2.222


def test__when_nothing_changed__should_return_current_pos_as_start_pos() -> None:
    instance_to_test = Localizer(start_lat=1.111, start_lon=2.222)
    actual = instance_to_test.get_current_position()

    assert actual["lat"] == 1.111
    assert actual["lon"] == 2.222


def test__when_new_fix__should_return_that_one_as_current_pos() -> None:
    instance_to_test = Localizer(start_lat=1.111, start_lon=2.222)

    new_fix: LatLon = LatLon(lat=8.0, lon=9.0)

    instance_to_test.update_with_new_fix(new_fix)

    actual = instance_to_test.get_current_position()

    assert actual["lat"] == new_fix["lat"]
    assert actual["lon"] == new_fix["lon"]


def test__when_instantiated__should_have_speed_zero() -> None:
    instance_to_test = Localizer()
    assert instance_to_test._current_speed_mps == 0.0


def test__when_instantiated__should_have_heading_zero() -> None:
    instance_to_test = Localizer()
    assert instance_to_test._current_heading == 0.0


def test__when_speed_zero__should_not_change_position() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(0.0)
    instance_to_test.calculate_position(10.0)
    assert instance_to_test.get_current_position()["lat"] == 0.0
    assert instance_to_test.get_current_position()["lon"] == 0.0


def test__when_speed_zero__should_not_travel_any_distance() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(0.0)
    instance_to_test.calculate_position(10.0)
    assert instance_to_test._meters_traveled == 0.0


def test__when_speed_1_mps__should_travel_1_m_after_1_second() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.calculate_position(1.0)
    assert instance_to_test._meters_traveled == 1.0


def test__when_speed_1_mps_and_heading_zero__should_keep_same_lon() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.calculate_position(1.0)
    actual = instance_to_test.get_current_position()
    assert actual["lon"] == 0.0


def test__when_speed_1_mps_and_heading_zero__lat_should_be_slightly_bigger() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.calculate_position(1.0)
    actual = instance_to_test.get_current_position()
    assert 9.043695e-06 > actual["lat"] > 9.043693e-060


def test__when_speed_1_mps_and_heading_90__should_keep_same_lat() -> None:
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(90)
    instance_to_test.calculate_position(1.0)
    actual = instance_to_test.get_current_position()
    assert actual["lat"] == 0.0


def test__when_speed_1_mps_and_heading_45__lat_and_lon_should_be_slightly_bigger() -> (
    None
):
    instance_to_test = Localizer()
    instance_to_test.update_speed_mps(1.0)
    instance_to_test.update_heading(45)
    instance_to_test.calculate_position(1.0)
    actual = instance_to_test.get_current_position()
    assert 6.3948579e-06 > actual["lat"] > 6.3948578e-06
    assert 6.3520483e-06 > actual["lon"] > 6.3520482e-06
