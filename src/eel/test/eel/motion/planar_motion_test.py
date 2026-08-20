from math import isclose

from eel.motion.planar_motion import (
    DEFAULT_FORWARD_CRUISE_MPS,
    DEFAULT_REVERSE_CRUISE_MPS,
    motor_to_speed_mps,
    planar_delta_m,
)


def test__when_motor_forward_full__should_use_forward_cruise() -> None:
    assert motor_to_speed_mps(1.0) == DEFAULT_FORWARD_CRUISE_MPS


def test__when_motor_reverse_full__should_use_reverse_cruise() -> None:
    assert motor_to_speed_mps(-1.0) == -DEFAULT_REVERSE_CRUISE_MPS


def test__when_heading_zero_and_forward__should_advance_along_x() -> None:
    dx, dy = planar_delta_m(1.0, 0.0, 1.0)
    assert isclose(dx, DEFAULT_FORWARD_CRUISE_MPS)
    assert isclose(dy, 0.0)


def test__when_heading_zero_and_reverse__should_move_along_negative_x() -> None:
    dx, dy = planar_delta_m(-1.0, 0.0, 1.0)
    assert isclose(dx, -DEFAULT_REVERSE_CRUISE_MPS)
    assert isclose(dy, 0.0)
