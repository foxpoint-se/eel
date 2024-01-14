import serial
import struct
import time

from rclpy.node import Node


FLOAT_SIZE_IN_BYTES = 4


class PressureSensor:
    def __init__(self, parent_node: Node) -> None:
        self.logger = parent_node.get_logger()
        self.serial_connection = serial.Serial("/dev/ttyUSB0", timeout=0)

        self.logger.info(f"{self.serial_connection.in_waiting} bytes in buffer, flushing now.")

        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()

        # Sleep time to fill the buffer with at least one message
        time.sleep(0.2)

        self.atmosphere_offset = 0
        self.atmosphere_offset = self.get_current_depth()

        self.logger.info(f"Sensor initialized, fluid density set to 997 km/m3, offset is {self.atmosphere_offset} m")

    def get_current_depth(self):
        depth_as_bytes = self.serial_connection.read(FLOAT_SIZE_IN_BYTES)

        return struct.unpack("f", depth_as_bytes)[0] - self.atmosphere_offset 
