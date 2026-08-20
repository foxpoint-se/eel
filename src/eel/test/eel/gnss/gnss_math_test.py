from eel.gnss.gnss_math import is_valid_gnss_latlon


def test__when_latlon_near_zero__should_be_invalid() -> None:
    assert not is_valid_gnss_latlon(0.0, 0.0)
    assert not is_valid_gnss_latlon(0.05, 17.9)


def test__when_latlon_is_real_fix__should_be_valid() -> None:
    assert is_valid_gnss_latlon(59.309395, 17.974279)
