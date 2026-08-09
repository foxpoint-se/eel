import math

from eel.pressure.pressure_math import calculate_center_depth, get_depth_velocity


def test__when_pitch_is_zero__should_return_sensor_depth() -> None:
    assert calculate_center_depth(2.0, 0.0) == 2.0


def test__when_pitch_ninety__should_apply_lever_arm() -> None:
    expected = 2.0 - (0.375 * math.sin(math.radians(90.0)))
    assert calculate_center_depth(2.0, 90.0, 0.375) == expected


def test__when_no_previous_sample__should_report_zero_velocity() -> None:
    assert get_depth_velocity(1.0, None, now=10.0, previous_depth_at=None) == 0.0


def test__when_depth_increases_over_time__should_report_positive_velocity() -> None:
    assert get_depth_velocity(2.0, previous_depth=1.0, now=2.0, previous_depth_at=1.0) == 1.0
