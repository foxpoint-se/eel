from math import cos, isclose, radians

from eel.motion.planar_motion import DEFAULT_FORWARD_CRUISE_MPS, DEFAULT_REVERSE_CRUISE_MPS
from eel_world_sim.motion_model import MotionModel


def test__when_dt_is_zero__should_not_move() -> None:
    model = MotionModel()
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    x, y = model.step(0.0)
    assert x == 0.0
    assert y == 0.0


def test__when_motor_idle__should_not_move() -> None:
    model = MotionModel()
    model.set_motor_cmd(0.0)
    model.set_heading_deg(90.0)
    x, y = model.step(1.0)
    assert x == 0.0
    assert y == 0.0


def test__when_motor_on_heading_zero__should_advance_along_x() -> None:
    model = MotionModel()
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    x, y = model.step(1.0)
    assert isclose(x, DEFAULT_FORWARD_CRUISE_MPS)
    assert isclose(y, 0.0)


def test__when_motor_on_heading_ninety__should_advance_along_y() -> None:
    model = MotionModel()
    model.set_motor_cmd(1.0)
    model.set_heading_deg(90.0)
    x, y = model.step(1.0)
    assert isclose(x, 0.0, abs_tol=1e-9)
    assert isclose(y, DEFAULT_FORWARD_CRUISE_MPS)


def test__when_motor_reverse_heading_zero__should_move_along_negative_x() -> None:
    model = MotionModel()
    model.set_motor_cmd(-1.0)
    model.set_heading_deg(0.0)
    x, y = model.step(1.0)
    assert isclose(x, -DEFAULT_REVERSE_CRUISE_MPS)
    assert isclose(y, 0.0)


def test__when_pitched_down__should_reduce_horizontal_travel() -> None:
    model = MotionModel()
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    model.set_pitch_deg(45.0)
    x, y = model.step(1.0)
    expected = DEFAULT_FORWARD_CRUISE_MPS * cos(radians(45.0))
    assert isclose(x, expected)
    assert isclose(y, 0.0)
