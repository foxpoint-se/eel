import RPi.GPIO as GPIO
from .pump_state_control import PumpStateControl

FILL_GPIO_LEVEL = GPIO.HIGH
EMPTY_GPIO_LEVEL = GPIO.LOW

RUN_GPIO_LEVEL = GPIO.HIGH
STOP_GPIO_LEVEL = GPIO.LOW


class PumpMotorControl(PumpStateControl):
    def __init__(self, motor_pin, direction_pin) -> None:
        self.motor_pin = motor_pin
        self.direction_pin = direction_pin

        # Initialize PI pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.motor_pin, GPIO.OUT)

    def _set_filling_up(self):
        GPIO.output(self.direction_pin, FILL_GPIO_LEVEL)

    def _set_emptying(self):
        GPIO.output(self.direction_pin, EMPTY_GPIO_LEVEL)

    def _start_motor(self):
        GPIO.output(self.motor_pin, RUN_GPIO_LEVEL)

    def _stop_motor(self):
        GPIO.output(self.motor_pin, STOP_GPIO_LEVEL)

    def stop(self):
        super().stop()
        self._stop_motor()

    def fill(self):
        super().fill()
        self._set_filling_up()
        self._start_motor()

    def empty(self):
        super().empty()
        self._set_emptying()
        self._start_motor()
