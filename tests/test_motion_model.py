from math import cos, isclose, radians

from eel.motion.planar_motion import DEFAULT_FORWARD_CRUISE_MPS, DEFAULT_REVERSE_CRUISE_MPS, DEFAULT_THROTTLE_EXPONENT
from eel_world_sim.motion_model import SIM_SPEED_RESPONSE_RATE_PER_S, MotionModel


def test__when_instantiated__should_use_sim_defaults() -> None:
    model = MotionModel()
    assert model.throttle_exponent == DEFAULT_THROTTLE_EXPONENT
    assert model.speed_response_rate_per_s == SIM_SPEED_RESPONSE_RATE_PER_S


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
    model.set_speed_response_rate_per_s(0.0)
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    x, y = model.step(1.0)
    assert isclose(x, DEFAULT_FORWARD_CRUISE_MPS)
    assert isclose(y, 0.0)


def test__when_motor_on_heading_ninety__should_advance_along_y() -> None:
    model = MotionModel()
    model.set_speed_response_rate_per_s(0.0)
    model.set_motor_cmd(1.0)
    model.set_heading_deg(90.0)
    x, y = model.step(1.0)
    assert isclose(x, 0.0, abs_tol=1e-9)
    assert isclose(y, DEFAULT_FORWARD_CRUISE_MPS)


def test__when_motor_reverse_heading_zero__should_move_along_negative_x() -> None:
    model = MotionModel()
    model.set_speed_response_rate_per_s(0.0)
    model.set_motor_cmd(-1.0)
    model.set_heading_deg(0.0)
    x, y = model.step(1.0)
    assert isclose(x, -DEFAULT_REVERSE_CRUISE_MPS)
    assert isclose(y, 0.0)


def test__when_pitched_down__should_reduce_horizontal_travel() -> None:
    model = MotionModel()
    model.set_speed_response_rate_per_s(0.0)
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    model.set_pitch_deg(45.0)
    x, y = model.step(1.0)
    expected = DEFAULT_FORWARD_CRUISE_MPS * cos(radians(45.0))
    assert isclose(x, expected)
    assert isclose(y, 0.0)


def test__when_exponent_is_two_and_half_throttle__should_reduce_travel() -> None:
    model = MotionModel()
    model.set_speed_response_rate_per_s(0.0)
    model.set_motor_cmd(0.5)
    model.set_heading_deg(0.0)
    model.set_throttle_exponent(2.0)
    x, y = model.step(1.0)
    assert isclose(x, DEFAULT_FORWARD_CRUISE_MPS * 0.25)
    assert isclose(y, 0.0)


def test__when_motor_stops__should_coast_before_stopping() -> None:
    model = MotionModel()
    model.set_speed_response_rate_per_s(1.0)
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    model.step(1.0)
    model.set_motor_cmd(0.0)
    x, _ = model.step(0.1)
    assert x > 0.0


def test__when_pitch_increases_while_coasting__should_reduce_horizontal_travel() -> None:
    model = MotionModel()
    model.set_speed_response_rate_per_s(1.0)
    model.set_motor_cmd(1.0)
    model.set_heading_deg(0.0)
    model.step(1.0)
    x_before = model.x_m
    model.set_motor_cmd(0.0)
    model.set_pitch_deg(45.0)
    model.step(0.1)
    dx_pitched = model.x_m - x_before

    model_flat = MotionModel()
    model_flat.set_speed_response_rate_per_s(1.0)
    model_flat.set_motor_cmd(1.0)
    model_flat.set_heading_deg(0.0)
    model_flat.step(1.0)
    x_before_flat = model_flat.x_m
    model_flat.set_motor_cmd(0.0)
    model_flat.step(0.1)
    dx_flat = model_flat.x_m - x_before_flat

    assert dx_pitched < dx_flat
    assert isclose(dx_pitched, dx_flat * cos(radians(45.0)))
