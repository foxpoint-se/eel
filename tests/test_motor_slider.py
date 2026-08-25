from eel_world_sim.gui.motor_slider import motor_slider_to_cmd


def test__when_slider_in_deadzone__should_return_zero() -> None:
    assert motor_slider_to_cmd(0.0) == 0.0
    assert motor_slider_to_cmd(5.0) == 0.0
    assert motor_slider_to_cmd(-5.0) == 0.0


def test__when_slider_at_full_forward__should_return_one() -> None:
    assert motor_slider_to_cmd(100.0) == 1.0


def test__when_slider_at_full_reverse__should_return_minus_one() -> None:
    assert motor_slider_to_cmd(-100.0) == -1.0


def test__when_slider_at_fifty_two__should_return_half() -> None:
    assert motor_slider_to_cmd(52.5) == 0.5
