import time
import ms5837
from rclpy.node import Node

# meters
CALIBRATION_DEPTH_1 = 0
CALIBRATION_DEPTH_2 = 0.08


class PressureSensor:
    def __init__(self, parent_node: Node) -> None:
        self.sensor = ms5837.MS5837_30BA()
        if not self.sensor.init():
            raise RuntimeError("Pressure sensor could not be initialized")

        if not self.sensor.read():
            raise RuntimeError("Pressure sensor cannot read")

        self.logger = parent_node.get_logger()
        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
        self.depth_1_value = None
        self.depth_2_value = None

        self._calibrate()

    def _get_depth_reading(self):
        self.sensor.read()
        return self.sensor.depth()

    def _get_calibrated_depth_reading(self):
        raw_reading = self._get_depth_reading()
        calibrated = (
            (raw_reading - self.depth_1_value)
            / (self.depth_2_value - self.depth_1_value)
            * (CALIBRATION_DEPTH_2 - CALIBRATION_DEPTH_1)
        )
        return calibrated

    def _calibrate(self):
        self.logger.info("Calibrating pressure sensor.")
        self.logger.info(
            "First step: hold pressure sensor at {} m".format(CALIBRATION_DEPTH_1)
        )
        time.sleep(3)
        self.depth_1_value = self._get_depth_reading()
        self.logger.info(
            "Second step: Hold pressure sensor at {} m".format(CALIBRATION_DEPTH_2)
        )
        time.sleep(5)
        self.depth_2_value = self._get_depth_reading()
        self.logger.info("Calibration done!")
        self.logger.info(
            "Value at {} m: {}".format(CALIBRATION_DEPTH_1, self.depth_1_value)
        )
        self.logger.info(
            "Value at {} m: {}".format(CALIBRATION_DEPTH_2, self.depth_2_value)
        )

    def get_current_depth(self):
        return self._get_calibrated_depth_reading()
