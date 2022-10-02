import RPi.GPIO as GPIO

PWM_FREQUENCY = 19200
DEFAULT_PWM_PIN = 18
DEFAULT_DIR_PIN = 25

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
pwm_output = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)


class MotorControl:
    def __init__(self) -> None:
        # Power supply is providing 15 voltage, the motor that we are using can only handle 12 V.
        # Therefore we need to cap the pwm output signal in order to not damage the motor.
        self.input_voltage = 15
        self.motor_max_voltage = 12
        self.pwm_max = 100
        self.motor_ctl_level = (
            self.motor_max_voltage / self.input_voltage
        ) * self.pwm_max

    def forward(self, signal):
        GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)
        pwm_output.start(signal * self.motor_ctl_level)

    def backward(self, signal):
        GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
        pwm_output.start(signal * self.motor_ctl_level)

    def stop(self):
        pwm_output.stop()


motor = MotorControl()
