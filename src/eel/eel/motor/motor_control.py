PWM_FREQUENCY = 1000
DEFAULT_PWM_PIN = 12
# DEFAULT_DIR_PIN = 13 # <--- old
DEFAULT_DIR_PIN = 6


class MotorControl:
    # TODO: remove logger probably?
    def __init__(self, simulate=False, logger=None) -> None:
        self.simulate = simulate
        self.logger = logger

        if not simulate:
            import RPi.GPIO as GPIO

            self.FORWARD_LEVEL = GPIO.HIGH
            self.BACKWARD_LEVEL = GPIO.LOW
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
            GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
            self.gpio = GPIO
            self.pwm_output = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)

    def forward(self, speed=100):
        if self.simulate:
            self.logger and self.logger.info(
                "Simulating going forward, speed {}".format(speed)
            )
        else:
            self.gpio.output(DEFAULT_DIR_PIN, self.FORWARD_LEVEL)
            self.pwm_output.start(speed)

    def backward(self, speed=100):
        if self.simulate:
            self.logger and self.logger.info(
                "Simulating going backward, speed {}".format(speed)
            )
        else:
            self.gpio.output(DEFAULT_DIR_PIN, self.BACKWARD_LEVEL)
            self.pwm_output.start(speed)

    def stop(self):
        if self.simulate:
            self.logger and self.logger.info("Simulating stopping motor.")
        else:
            self.pwm_output.stop()
