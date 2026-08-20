from math import isclose

from eel_world_sim.tank_fill_model import (
    EMPTY_VELOCITY_PER_S,
    FILL_VELOCITY_PER_S,
    MAX_LEVEL,
    MIN_LEVEL,
    TankFillModel,
)


def test__when_dt_is_zero__should_not_change_level() -> None:
    model = TankFillModel()
    model.set_pump_cmd(1.0)
    assert model.step(0.0) == 0.0


def test__when_pump_fills__should_raise_level() -> None:
    model = TankFillModel()
    model.set_pump_cmd(1.0)
    assert isclose(model.step(1.0), FILL_VELOCITY_PER_S)


def test__when_pump_empties__should_lower_level() -> None:
    model = TankFillModel()
    model.level = 0.5
    model.set_pump_cmd(-1.0)
    assert isclose(model.step(1.0), 0.5 - EMPTY_VELOCITY_PER_S)


def test__when_pump_stopped__should_hold_level() -> None:
    model = TankFillModel()
    model.level = 0.4
    model.set_pump_cmd(0.0)
    assert model.step(1.0) == 0.4


def test__when_fill_would_exceed_full__should_cap() -> None:
    model = TankFillModel()
    model.level = MAX_LEVEL - 0.01
    model.set_pump_cmd(1.0)
    assert model.step(10.0) == MAX_LEVEL


def test__when_empty_would_go_negative__should_cap() -> None:
    model = TankFillModel()
    model.level = 0.01
    model.set_pump_cmd(-1.0)
    assert model.step(10.0) == MIN_LEVEL
