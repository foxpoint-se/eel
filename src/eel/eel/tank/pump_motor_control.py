import RPi.GPIO as GPIO


FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW


class PumpMotorControl:
    def __init__(self, motor_pin, direction_pin) -> None:
        self.motor_pin = motor_pin
        self.direction_pin = direction_pin
        self.is_on = False
        self.is_forward = False

        # Initialize PI pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.motor_pin, GPIO.OUT)

    def _set_forward(self):
        GPIO.output(self.direction_pin, FORWARD_LEVEL)
        self.is_forward = True

    def _set_backward(self):
        GPIO.output(self.direction_pin, BACKWARD_LEVEL)
        self.is_forward = False

    def _start(self):
        GPIO.output(self.motor_pin, FORWARD_LEVEL)
        self.is_on = True

    # ====

    def stop(self):
        GPIO.output(self.motor_pin, GPIO.LOW)
        self.is_on = False

    def fill(self):
        self._set_forward()
        self._start()

    def empty(self):
        self._set_backward()
        self._start()

    def get_is_forward(self):
        return self.is_forward

    def get_is_on(self):
        return self.is_on
