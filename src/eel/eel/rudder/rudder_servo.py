from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


class RudderServo:
    # TODO: remove logger probably?
    def __init__(self, simulate=False, logger=None) -> None:
        self.simulate = simulate
        self.logger = logger
        if not simulate:
            factory = PiGPIOFactory()
            self.servo = Servo(
                13,
                pin_factory=factory,
                min_pulse_width=0.81 / 1000,
                max_pulse_width=2.2 / 1000,
            )

    def set_value(self, value):
        if self.simulate:
            if self.logger:
                self.logger.info("Set rudder to {}".format(value))
        else:
            self.servo.value = value

    def detach(self):
        if self.simulate:
            if self.logger:
                self.logger.info("Simulating detaching servo.")
        else:
            self.servo.detach()
