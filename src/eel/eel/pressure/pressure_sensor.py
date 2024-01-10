import serial
import struct
from rclpy.node import Node


FLOAT_SIZE_IN_BYTES = 4


class PressureSensor:
    def __init__(self, parent_node: Node) -> None:
        self.serial_connection = serial.Serial("/dev/ttyUSB1")
        self.serial_connection.flushInput()
        self.serial_connection.flushOutput()

        self.atmosphere_offset = self.get_current_depth()

        self.logger = parent_node.get_logger()

        self.logger.info(f"Sensor initialized, fluid density set to 997 km/m3, atmosphere "
                         f"offset is {self.atmosphere_offset} m")

    def get_current_depth(self):
        depth_as_bytes = self.serial_connection.read(FLOAT_SIZE_IN_BYTES)

        return struct.unpack("f", depth_as_bytes)[0]
