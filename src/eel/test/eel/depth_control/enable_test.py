from eel.depth_control.enable import DepthEnable


def test__when_booting__should_not_be_enabled() -> None:
    assert DepthEnable().is_enabled is False


def test__when_depth_cmd_arrives__should_enable() -> None:
    enable = DepthEnable()
    enable.handle_depth_cmd()
    assert enable.is_enabled is True


def test__when_enabled_cmd_is_false__should_disable() -> None:
    enable = DepthEnable()
    enable.handle_depth_cmd()
    enable.handle_enabled_cmd(False)
    assert enable.is_enabled is False


def test__when_enabled_cmd_is_true__should_enable() -> None:
    enable = DepthEnable()
    enable.handle_enabled_cmd(True)
    assert enable.is_enabled is True
