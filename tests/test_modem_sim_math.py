from eel_world_sim.modem_sim_math import simulated_registration_status, simulated_signal_strength


def test__when_on_surface__should_have_full_signal() -> None:
    assert simulated_signal_strength(0.0) == 31
    assert simulated_registration_status(0.0) == 1


def test__when_deep__should_lose_signal() -> None:
    assert simulated_signal_strength(1.0) == 0
    assert simulated_registration_status(1.0) == 0
