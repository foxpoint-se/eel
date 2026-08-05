from eel.led_control.led_pulse import plan_pulse_steps


def test__when_two_pulses__should_alternate_on_off_with_wait_only_while_on() -> None:
    assert plan_pulse_steps(2, 0.5) == [
        ("on", 0.5),
        ("off", 0.0),
        ("on", 0.5),
        ("off", 0.0),
    ]


def test__when_zero_pulses__should_return_empty_plan() -> None:
    assert plan_pulse_steps(0, 0.5) == []


def test__when_negative_pulse_time__should_return_empty_plan() -> None:
    assert plan_pulse_steps(3, -0.1) == []
