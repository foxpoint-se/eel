"""Serial driver for the Bar02 depth stream (meters). No ROS."""

import serial

from .pressure_math import has_depth_sample
from .pressure_serial_frames import FLOAT32_SIZE, take_float32_le


class PressureSerialDriver:
    def __init__(self, serial_port: str) -> None:
        self._serial = serial.Serial(serial_port, timeout=0)
        self._rx_buffer = b""
        self._atmosphere_offset_m: float | None = None

        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    @property
    def is_calibrated(self) -> bool:
        return self._atmosphere_offset_m is not None

    def get_depth_m(self) -> float | None:
        """Depth in meters relative to the first good sample (surface offset)."""
        sample_m = self._read_depth_sample()
        if not has_depth_sample(sample_m):
            return None
        if self._atmosphere_offset_m is None:
            self._atmosphere_offset_m = sample_m
            return 0.0
        return sample_m - self._atmosphere_offset_m

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def _read_depth_sample(self) -> float | None:
        waiting = self._serial.in_waiting
        to_read = waiting if waiting > 0 else FLOAT32_SIZE
        self._rx_buffer += self._serial.read(to_read)
        values, self._rx_buffer = take_float32_le(self._rx_buffer)
        if not values:
            return None
        return values[-1]
