from eel.utils.pid_controller import OutputLimits, PidController, PidGains, PidState, pid_step


def test__when_saturated_with_back_calculation__should_not_wind_integral() -> None:
    state = PidState()
    gains = PidGains(kP=0.0, kI=1.0, kD=0.0)
    limits = OutputLimits(min=0.0, max=1.0)

    for _ in range(20):
        output, state = pid_step(state, measured=0.0, setpoint=10.0, gains=gains, dt=1.0, limits=limits)
        assert output == 1.0

    assert state.cumulative_error < 2.0


def test__when_no_limits__should_return_unclamped_output() -> None:
    state = PidState()
    gains = PidGains(kP=2.0, kI=0.0, kD=0.0)

    output, _ = pid_step(state, measured=1.0, setpoint=5.0, gains=gains, dt=0.1, limits=None)

    assert output == 8.0


def test__when_output_clamped__should_respect_limits() -> None:
    state = PidState()
    gains = PidGains(kP=10.0, kI=0.0, kD=0.0)
    limits = OutputLimits(min=-1.0, max=1.0)

    output, _ = pid_step(state, measured=0.0, setpoint=10.0, gains=gains, dt=0.1, limits=limits)

    assert output == 1.0


def test__when_ki_zero_and_clamped__should_clamp_without_using_integral() -> None:
    state = PidState()
    gains = PidGains(kP=50.0, kI=0.0, kD=0.0)
    limits = OutputLimits(min=-1.0, max=1.0)

    output, state = pid_step(state, measured=0.0, setpoint=10.0, gains=gains, dt=1.0, limits=limits)

    assert output == 1.0
    assert state.cumulative_error == 10.0


def test__when_dt_zero_and_clamped__should_not_back_calculate_integral() -> None:
    state = PidState()
    gains = PidGains(kP=10.0, kI=1.0, kD=0.0)
    limits = OutputLimits(min=-1.0, max=1.0)

    output, state = pid_step(state, measured=0.0, setpoint=10.0, gains=gains, dt=0.0, limits=limits)

    assert output == 1.0
    assert state.cumulative_error == 0.0


def test__when_output_limits_are_swapped__should_raise() -> None:
    pid = PidController(set_point=0.0)
    try:
        pid.update_output_limits(1.0, -1.0)
        raise AssertionError("expected ValueError")
    except ValueError:
        pass
