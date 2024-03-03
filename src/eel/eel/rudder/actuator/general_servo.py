from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
from rclpy.logging import get_logger
from .types import ServoOptions

logger = get_logger(__name__)


def create_pigpio_factory(host: str, retries: int, sleep_time: int) -> PiGPIOFactory:
    attempts = 0
    factory = None
    while attempts < retries and factory is None:
        try:
            factory = PiGPIOFactory(host=host, port=8888)
        except OSError:
            factory = None
            logger.info("Could not create pigpio factory, trying again")
            sleep(sleep_time)
        finally:
            retries += 1
    return factory


class RudderServo:
    def __init__(
        self,
        options: ServoOptions,
        pigpiod_host: str,
    ) -> None:
        factory = create_pigpio_factory(host=pigpiod_host, retries=5, sleep_time=3)
        if not factory:
            raise ValueError("Could not create pigpio factory, exiting")
        self.servo = Servo(
            options["pin"],
            pin_factory=factory,
            min_pulse_width=options["min_pulse_width"],
            max_pulse_width=options["max_pulse_width"],
        )
        self.flip_direction = options["flip_direction"]
        self.cap_min = options["cap_min"]
        self.cap_max = options["cap_max"]

    # We want -1 to be left and 1 to be right, but for some reason it has been
    # flipped on the servo. So we just flip it back by setting it to its inverse.
    def set_value(self, value):
        if value > self.cap_max:
            value = self.cap_max
        if value < self.cap_min:
            value = self.cap_min
        if self.flip_direction:
            value = -value
        self.servo.value = value

    def detach(self):
        self.servo.detach()
