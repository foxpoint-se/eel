from eel.tank.tank_topics import tank_level_topic, tank_pump_setpoint_topic


def test__when_status_topic__should_derive_mid_topics() -> None:
    assert tank_level_topic("tank_front/status") == "tank_front/level"
    assert tank_pump_setpoint_topic("tank_front/status") == "tank_front/pump_setpoint"


def test__when_status_topic_missing_suffix__should_raise() -> None:
    try:
        tank_level_topic("tank_front")
    except ValueError:
        return
    raise AssertionError("expected ValueError")
