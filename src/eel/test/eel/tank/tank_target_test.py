import pytest
from eel.tank.tank_target import (
    RunningAverage,
    TankTargetConfig,
    is_at_ceiling,
    is_at_floor,
    is_within_accepted_target_boundaries,
    tank_stop_reason,
)

# 0–100 scale with round margins so tests read without hidden constants.
TEST_CONFIG = TankTargetConfig(
    level_floor=0.0,
    level_ceiling=100.0,
    edge_margin=10.0,
    target_half_band=5.0,
)


def test__when_level_at_target__should_be_within_boundaries() -> None:
    assert is_within_accepted_target_boundaries(50.0, 50.0, TEST_CONFIG)


def test__when_level_is_none__should_not_be_within_boundaries() -> None:
    assert not is_within_accepted_target_boundaries(None, 50.0, TEST_CONFIG)


def test__when_level_within_edge_margin_of_floor__should_be_at_floor() -> None:
    assert is_at_floor(10.0, TEST_CONFIG)


def test__when_level_within_edge_margin_of_ceiling__should_be_at_ceiling() -> None:
    assert is_at_ceiling(90.0, TEST_CONFIG)


def test__when_at_floor_and_target_not_above_level__should_stop_for_floor() -> None:
    assert tank_stop_reason(level_average=0.0, target_level=0.0, config=TEST_CONFIG) == "floor_reached"


def test__when_at_ceiling_and_target_not_below_level__should_stop_for_ceiling() -> None:
    assert tank_stop_reason(level_average=100.0, target_level=100.0, config=TEST_CONFIG) == "ceiling_reached"


def test__when_within_target_band__should_stop_for_target() -> None:
    assert tank_stop_reason(level_average=50.0, target_level=50.0, config=TEST_CONFIG) == "target_reached"


def test__when_still_adjusting__should_return_no_stop_reason() -> None:
    assert tank_stop_reason(level_average=20.0, target_level=80.0, config=TEST_CONFIG) is None


def test__when_running_average_filled__should_return_mean() -> None:
    avg = RunningAverage(2)
    avg.add_sample(0.0)
    avg.add_sample(100.0)
    assert avg.get_average() == 50.0


def test__when_running_average_size_is_zero__should_raise() -> None:
    with pytest.raises(ValueError):
        RunningAverage(0)
