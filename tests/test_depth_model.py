from math import isclose, radians, sin

from eel_world_sim.depth_model import (
    FLOAT_VELOCITY_MPS,
    MAX_DEPTH_M,
    MIN_DEPTH_M,
    TERMINAL_VELOCITY_MPS,
    DepthModel,
)


def test__when_dt_is_zero__should_not_change_depth() -> None:
    model = DepthModel()
    model.set_motor_cmd(1.0)
    model.set_pitch_deg(45.0)
    assert model.step(0.0) == 0.0


def test__when_motor_idle__should_float_up_slowly() -> None:
    model = DepthModel()
    model.depth_m = 1.0
    model.set_motor_cmd(0.0)
    depth = model.step(1.0)
    assert isclose(depth, 1.0 + FLOAT_VELOCITY_MPS)


def test__when_diving_nose_down__should_go_deeper() -> None:
    model = DepthModel()
    model.set_motor_cmd(1.0)
    model.set_pitch_deg(45.0)
    dive = sin(radians(45.0)) * TERMINAL_VELOCITY_MPS
    expected = FLOAT_VELOCITY_MPS + dive
    assert isclose(model.step(1.0), expected)


def test__when_pitch_near_vertical__should_not_explode_dive_rate() -> None:
    model = DepthModel()
    model.set_motor_cmd(1.0)
    model.set_pitch_deg(89.0)
    dive = sin(radians(89.0)) * TERMINAL_VELOCITY_MPS
    expected = FLOAT_VELOCITY_MPS + dive
    assert isclose(model.step(1.0), expected)
    assert dive < TERMINAL_VELOCITY_MPS * 2.0


def test__when_depth_would_exceed_max__should_cap() -> None:
    model = DepthModel()
    model.depth_m = MAX_DEPTH_M - 0.01
    model.set_motor_cmd(1.0)
    model.set_pitch_deg(80.0)
    assert model.step(10.0) == MAX_DEPTH_M


def test__when_depth_would_go_negative__should_cap_at_surface() -> None:
    model = DepthModel()
    model.depth_m = 0.01
    model.set_motor_cmd(0.0)
    assert model.step(10.0) == MIN_DEPTH_M
