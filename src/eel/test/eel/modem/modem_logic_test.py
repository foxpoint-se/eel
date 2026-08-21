from eel.modem.modem_logic import has_modem_readings, modem_connectivity


def test__when_readings_missing__should_not_have_modem_readings() -> None:
    assert not has_modem_readings(None, 20)
    assert not has_modem_readings(1, None)


def test__when_readings_present_including_zero__should_have_modem_readings() -> None:
    assert has_modem_readings(0, 0)
    assert has_modem_readings(1, 15)


def test__when_not_registered__should_not_report_connectivity() -> None:
    assert not modem_connectivity(reg_status=0, signal_strength=20, ping_ok=True)


def test__when_registered_but_signal_weak__should_not_report_connectivity() -> None:
    assert not modem_connectivity(reg_status=1, signal_strength=10, ping_ok=True)


def test__when_registered_and_ping_ok__should_report_connectivity() -> None:
    assert modem_connectivity(reg_status=1, signal_strength=11, ping_ok=True)


def test__when_registered_but_ping_fails__should_not_report_connectivity() -> None:
    assert not modem_connectivity(reg_status=1, signal_strength=11, ping_ok=False)
