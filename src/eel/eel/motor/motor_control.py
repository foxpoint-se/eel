import RPi.GPIO as GPIO

PWM_FREQUENCY = 1000
DEFAULT_PWM_PIN = 12
DEFAULT_DIR_PIN = 20


class MotorControl:
    def __init__(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
        GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)

        # Power supply is providing 15 voltage, the motor that we are using can only handle 12 V.
        # Therefore we need to cap the pwm output signal in order to not damage the motor.
        self.input_voltage = 15
        self.motor_max_voltage = 12
        self.pwm_max = 100
        self.motor_ctl_level = (self.motor_max_voltage / self.input_voltage) * self.pwm_max

    def forward(self, signal: float) -> None:
        GPIO.output(DEFAULT_DIR_PIN, GPIO.LOW)
        self._pwm.start(signal * self.motor_ctl_level)

    def backward(self, signal: float) -> None:
        GPIO.output(DEFAULT_DIR_PIN, GPIO.HIGH)
        self._pwm.start(signal * self.motor_ctl_level)

    def stop(self) -> None:
        self._pwm.stop()

    def close(self) -> None:
        self.stop()
        GPIO.cleanup((DEFAULT_DIR_PIN, DEFAULT_PWM_PIN))
