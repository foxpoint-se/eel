from unittest.mock import MagicMock, patch

from eel.pressure.pressure_serial_driver import PressureSerialDriver


@patch("eel.pressure.pressure_serial_driver.serial.Serial")
def test__when_first_frame_is_nan__should_not_calibrate(_serial: MagicMock) -> None:
    driver = PressureSerialDriver("/dev/ttyUSB0")
    driver._read_depth_sample = MagicMock(return_value=float("nan"))

    assert driver.get_depth_m() is None
    assert not driver.is_calibrated

    driver._read_depth_sample.return_value = 2.0
    assert driver.get_depth_m() == 0.0
    assert driver.is_calibrated

    driver._read_depth_sample.return_value = 2.5
    assert driver.get_depth_m() == 0.5
