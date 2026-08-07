import time

import serial
from rclpy.node import Node

from .pressure_serial_frames import FLOAT32_SIZE, take_float32_le
from .pressure_source import PressureSource


class PressureSensor(PressureSource):
    def __init__(self, parent_node: Node, serial_port: str) -> None:
        self.logger = parent_node.get_logger()

        self.serial_connection = serial.Serial(serial_port, timeout=0)
        self._rx_buffer = b""

        self.logger.info(f"{self.serial_connection.in_waiting} bytes in buffer, flushing now.")

        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()

        number_of_retries = 20

        self.atmosphere_offset = self._get_initial_depth(retries=number_of_retries)

        if self.atmosphere_offset is None:
            raise RuntimeError(f"Could not determine atmosphere offset after {number_of_retries} retries.")

        self.logger.info(f"Sensor initialized, fluid density set to 997 kg/m3, offset is {self.atmosphere_offset} m")

    def _get_initial_depth(self, retries: int = 10, sleep_time: float = 0.2) -> float | None:
        for _ in range(retries):
            depth = self._get_depth_reading()
            if depth is not None:
                return depth
            self.logger.info("Retrying getting initial depth...")
            time.sleep(sleep_time)
        return None

    def _get_depth_reading(self) -> float | None:
        waiting = self.serial_connection.in_waiting
        to_read = waiting if waiting > 0 else FLOAT32_SIZE
        self._rx_buffer += self.serial_connection.read(to_read)
        values, self._rx_buffer = take_float32_le(self._rx_buffer)
        if not values:
            return None
        return values[-1]

    def get_current_depth(self) -> float | None:
        depth = self._get_depth_reading()
        if depth is not None and self.atmosphere_offset is not None:
            return depth - self.atmosphere_offset
        return None
