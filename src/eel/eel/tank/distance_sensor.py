from rclpy.node import Node
import RPi.GPIO as GPIO
import adafruit_vl53l0x
import board
import busio
import time


DEFAULT_I2C_ADDRESS = 29

ON_LEVEL = GPIO.HIGH
OFF_LEVEL = GPIO.LOW


def wait_for_address_change(xshut_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(xshut_pin, GPIO.OUT)
    GPIO.output(xshut_pin, OFF_LEVEL)
    time.sleep(2)
    GPIO.output(xshut_pin, ON_LEVEL)


class DistanceSensor:
    def __init__(
        self, address, timing_budget=50000, xshut_pin=0, parent_node: Node = None
    ):
        self.address = address
        self.xshut_pin = xshut_pin
        # self.resolution = (
        #     0.0  # TODO Find sensor resolution, used to determine safe max/min limit
        # )

        self.parent_node = parent_node

        i2c = busio.I2C(board.SCL, board.SDA)

        if self.xshut_pin:
            wait_for_address_change(self.xshut_pin)

        try:
            self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
            self.parent_node.get_logger().info(
                "Setting address to: {}".format(self.address)
            )
            self.vl53.set_address(self.address)
        except:
            self.vl53 = adafruit_vl53l0x.VL53L0X(i2c, self.address)

        self.vl53.measurement_timing_budget = timing_budget

    # @property
    # def resolution(self):
    #     return self.resolution

    # def set_address(self):
    #     pass

    def get_range(self):
        current_range = self.vl53.range
        # self.parent_node.get_logger().info("Getting range: {}".format(current_range))
        return current_range
