from math import isclose

from eel_world_sim.attitude_model import (
    ANGULAR_VELOCITY_DEGPS,
    MAX_PITCH_DEG,
    TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS,
    AttitudeModel,
    momentum_difference,
)


def test__when_dt_is_zero__should_not_change_attitude() -> None:
    model = AttitudeModel()
    model.set_motor_cmd(1.0)
    model.set_rudder(0.5, 0.5)
    heading, pitch = model.step(0.0)
    assert heading == 0.0
    assert pitch == 0.0


def test__when_motor_and_rudder_y__should_pitch() -> None:
    model = AttitudeModel()
    model.set_motor_cmd(1.0)
    model.set_rudder(0.0, 1.0)
    _, pitch = model.step(1.0)
    assert isclose(pitch, TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS)


def test__when_motor_idle__should_not_pitch_from_rudder() -> None:
    model = AttitudeModel()
    model.set_motor_cmd(0.0)
    model.set_rudder(0.0, 1.0)
    _, pitch = model.step(1.0)
    assert pitch == 0.0


def test__when_pitch_would_exceed_cap__should_cap() -> None:
    model = AttitudeModel()
    model.pitch_deg = MAX_PITCH_DEG - 0.1
    model.set_motor_cmd(1.0)
    model.set_rudder(0.0, 1.0)
    _, pitch = model.step(10.0)
    assert pitch == MAX_PITCH_DEG


def test__when_tanks_unbalanced__should_change_pitch_without_motor() -> None:
    model = AttitudeModel()
    model.set_tank_levels(front_level=1.0, rear_level=0.0)
    assert isclose(momentum_difference(1.0, 0.0), 1.0)
    _, pitch = model.step(1.0)
    assert isclose(pitch, TERMINAL_PITCH_ANGULAR_VELOCITY_DEGPS)


def test__when_rudder_right_cmd__should_increase_compass_heading() -> None:
    model = AttitudeModel()
    model.set_motor_cmd(1.0)
    model.set_rudder(1.0, 0.0)
    heading, _ = model.step(1.0)
    assert isclose(heading, ANGULAR_VELOCITY_DEGPS)


def test__when_rudder_left_cmd__should_decrease_compass_heading() -> None:
    model = AttitudeModel()
    model.set_motor_cmd(1.0)
    model.set_rudder(-1.0, 0.0)
    heading, _ = model.step(1.0)
    assert isclose(heading, 360.0 - ANGULAR_VELOCITY_DEGPS)
