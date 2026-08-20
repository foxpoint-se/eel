from eel.rudder.rudder_math import roll_compensated_deflection, servo_command_with_offset


def test__when_no_roll__should_keep_deflection() -> None:
    out = roll_compensated_deflection({"x": 0.5, "y": -0.25}, 0.0)
    assert out["x"] == 0.5
    assert out["y"] == -0.25


def test__when_offset_applied__should_clamp_to_cap() -> None:
    assert servo_command_with_offset(0.8, 0.2, -0.75, 0.75) == 0.75
    assert servo_command_with_offset(-0.5, -0.4, -0.75, 0.75) == -0.75
