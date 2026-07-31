from eel.imu.imu_sensor import parse_euler_reading, update_pitch_velocity_state


def test__when_heading_is_none__should_return_none() -> None:
    assert parse_euler_reading(None, 1.0, 2.0) is None


def test__when_roll_is_none__should_return_none() -> None:
    assert parse_euler_reading(90.0, None, 2.0) is None


def test__when_pitch_is_none__should_return_none() -> None:
    assert parse_euler_reading(90.0, 1.0, None) is None


def test__when_valid_reading__should_apply_mount_correction() -> None:
    result = parse_euler_reading(0.0, 0.0, 0.0)

    assert result is not None
    heading, roll, pitch = result
    assert heading == 180.0
    assert roll == 0.0
    assert pitch == 1.69


def test__when_euler_missing__should_clear_pitch_cache() -> None:
    velocity, pitch, pitch_at = update_pitch_velocity_state(
        euler=None,
        now=100.0,
        previous_pitch=1.0,
        previous_pitch_at=99.0,
    )

    assert velocity is None
    assert pitch is None
    assert pitch_at is None


def test__when_euler_returns_after_gap__should_have_zero_velocity() -> None:
    velocity, pitch, pitch_at = update_pitch_velocity_state(
        euler=(0.0, 0.0, 5.0),
        now=100.0,
        previous_pitch=None,
        previous_pitch_at=None,
    )

    assert velocity == 0.0
    assert pitch == 5.0
    assert pitch_at == 100.0
