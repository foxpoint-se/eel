from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
from rclpy.logging import get_logger

logger = get_logger(__name__)

# horizontal pin: 13
# horizontal min pulse width: 0.81 / 1000
# horizontal max pulse width: 2.2 / 1000
# horizontal flip direction: True


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
        pin: int,
        max_pulse_width: float,
        min_pulse_width: float,
        flip_direction: bool,
        cap_min: float = -1.0,
        cap_max: float = 1.0,
        pigpiod_host: str = "localhost",
    ) -> None:
        factory = create_pigpio_factory(host=pigpiod_host, retries=5, sleep_time=3)
        if not factory:
            raise ValueError("Could not create pigpio factory, exiting")
        self.servo = Servo(
            pin,
            pin_factory=factory,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
        )
        self.flip_direction = flip_direction
        self.cap_min = cap_min
        self.cap_max = cap_max

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
