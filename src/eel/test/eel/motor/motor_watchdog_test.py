from eel.motor.motor_watchdog import command_is_stale, require_positive_cmd_timeout


def test__when_no_command_seen__should_not_be_stale() -> None:
    assert not command_is_stale(last_cmd_at=None, now=10.0, timeout_s=1.0)


def test__when_command_within_timeout__should_not_be_stale() -> None:
    assert not command_is_stale(last_cmd_at=9.5, now=10.0, timeout_s=1.0)


def test__when_command_older_than_timeout__should_be_stale() -> None:
    assert command_is_stale(last_cmd_at=8.0, now=10.0, timeout_s=1.0)


def test__when_timeout_is_positive__should_return_it() -> None:
    assert require_positive_cmd_timeout(1.0) == 1.0


def test__when_timeout_is_not_positive__should_raise() -> None:
    try:
        require_positive_cmd_timeout(0.0)
    except ValueError:
        return
    raise AssertionError("expected ValueError")
