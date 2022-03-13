from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


class RudderServo:
    def __init__(self) -> None:
        factory = PiGPIOFactory()
        self.servo = Servo(
            13,
            pin_factory=factory,
            min_pulse_width=0.81 / 1000,
            max_pulse_width=2.2 / 1000,
        )

    # We want -1 to be left and 1 to be right, but for some reason it has been
    # flipped on the servo. So we just flip it back by setting it to its inverse.
    def set_value(self, value):
        self.servo.value = -value

    def detach(self):
        self.servo.detach()
