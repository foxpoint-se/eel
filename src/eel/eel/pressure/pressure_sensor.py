import time
import ms5837
from rclpy.node import Node


class PressureSensor:
    def __init__(self, parent_node: Node) -> None:
        self.sensor = ms5837.MS5837_02BA()
        # TODO: try catch on startup, and try again in some way
        if not self.sensor.init():
            raise RuntimeError("Pressure sensor could not be initialized")

        if not self.sensor.read():
            raise RuntimeError("Pressure sensor cannot read")

        offset_readings = 0
        
        # nof_readings = 5
        # for _ in range(nof_readings):
        #     offset_readings += self.sensor.depth()

        self.atmosphere_offset = self.sensor.depth()

        self.logger = parent_node.get_logger()
        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        self.logger.info(f"Sensor initialized, fluid density set to {ms5837.DENSITY_FRESHWATER} km/m3, atmosphere offset is {self.atmosphere_offset} m")

    def get_current_depth(self):
        self.sensor.read()
        
        return self.sensor.depth() - self.atmosphere_offset 
