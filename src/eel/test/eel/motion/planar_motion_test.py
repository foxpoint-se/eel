from math import cos, isclose, radians

from eel.motion.planar_motion import (
    DEFAULT_FORWARD_CRUISE_MPS,
    DEFAULT_REVERSE_CRUISE_MPS,
    motor_to_speed_mps,
    pitch_horizontal_scale,
    planar_delta_m,
)


def test__when_motor_forward_full__should_use_forward_cruise() -> None:
    assert motor_to_speed_mps(1.0) == DEFAULT_FORWARD_CRUISE_MPS


def test__when_motor_reverse_full__should_use_reverse_cruise() -> None:
    assert motor_to_speed_mps(-1.0) == -DEFAULT_REVERSE_CRUISE_MPS


def test__when_pitch_is_zero__should_scale_to_one() -> None:
    assert pitch_horizontal_scale(0.0) == 1.0


def test__when_pitch_is_forty_five__should_scale_by_cosine() -> None:
    assert isclose(pitch_horizontal_scale(45.0), cos(radians(45.0)))


def test__when_pitch_is_steep__should_not_invert_horizontal_motion() -> None:
    assert pitch_horizontal_scale(100.0) == 0.0


def test__when_motor_forward_and_pitch_down__should_reduce_speed() -> None:
    assert isclose(motor_to_speed_mps(1.0, pitch_deg=45.0), DEFAULT_FORWARD_CRUISE_MPS * cos(radians(45.0)))


def test__when_heading_zero_and_forward__should_advance_along_x() -> None:
    dx, dy = planar_delta_m(1.0, 0.0, 1.0)
    assert isclose(dx, DEFAULT_FORWARD_CRUISE_MPS)
    assert isclose(dy, 0.0)


def test__when_heading_zero_and_reverse__should_move_along_negative_x() -> None:
    dx, dy = planar_delta_m(-1.0, 0.0, 1.0)
    assert isclose(dx, -DEFAULT_REVERSE_CRUISE_MPS)
    assert isclose(dy, 0.0)


def test__when_pitched_down_and_forward__should_reduce_planar_delta() -> None:
    dx, dy = planar_delta_m(1.0, 0.0, 1.0, pitch_deg=45.0)
    expected = DEFAULT_FORWARD_CRUISE_MPS * cos(radians(45.0))
    assert isclose(dx, expected)
    assert isclose(dy, 0.0)
