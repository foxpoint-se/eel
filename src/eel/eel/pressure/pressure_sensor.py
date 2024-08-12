from typing import Union, cast
import serial
import struct
import time
from rclpy.node import Node
from .pressure_source import PressureSource

FLOAT_SIZE_IN_BYTES = 4


class PressureSensor(PressureSource):
    def __init__(self, parent_node: Node, serial_port: str) -> None:
        self.logger = parent_node.get_logger()

        self.serial_connection = serial.Serial(serial_port, timeout=0)

        self.logger.info(f"{self.serial_connection.in_waiting} bytes in buffer, flushing now.")

        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()

        number_of_retries = 20

        self.atmosphere_offset = self._get_initial_depth(retries=number_of_retries)

        if not self.atmosphere_offset:
            raise Exception("Could not determine atmosphere offset after 10 retries.")

        self.logger.info(f"Sensor initialized, fluid density set to 997 km/m3, offset is {self.atmosphere_offset} m")

    def _get_initial_depth(self, retries: int = 10, sleep_time: float = 0.2) -> Union[float, None]:
        for i in range(retries):
            depth = self._get_depth_reading()
            if depth:
                return depth
            self.logger.info("Retrying getting initial depth...")
            time.sleep(sleep_time)
        return None

    def _get_depth_as_bytes(self) -> bytes:
        return self.serial_connection.read(FLOAT_SIZE_IN_BYTES)

    def _get_depth_from_bytes(self, value: bytes) -> Union[float, None]:
        if value:
            stuff = struct.unpack("f", value)
            depth = cast(float, stuff[0])
            return depth
        return None
        
    def _get_depth_reading(self) -> Union[float, None]:
        depth_as_bytes = self._get_depth_as_bytes()
        if not depth_as_bytes:
            return None
        return self._get_depth_from_bytes(depth_as_bytes)
    
    def get_current_depth(self) -> Union[float, None]:
        depth = self._get_depth_reading()
        if depth and self.atmosphere_offset:
            return depth - self.atmosphere_offset
