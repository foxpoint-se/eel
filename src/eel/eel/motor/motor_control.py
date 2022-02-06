import RPi.GPIO as GPIO

PWM_FREQUENCY = 1000
DEFAULT_PWM_PIN = 12
# DEFAULT_DIR_PIN = 13 # <--- old
DEFAULT_DIR_PIN = 6

FORWARD_LEVEL = GPIO.HIGH
BACKWARD_LEVEL = GPIO.LOW
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DEFAULT_DIR_PIN, GPIO.OUT)
GPIO.setup(DEFAULT_PWM_PIN, GPIO.OUT)
pwm_output = GPIO.PWM(DEFAULT_PWM_PIN, PWM_FREQUENCY)


class MotorControl:
    def __init__(self) -> None:
        pass

    def forward(self, speed=100):
        GPIO.output(DEFAULT_DIR_PIN, FORWARD_LEVEL)
        pwm_output.start(speed)

    def backward(self, speed=100):
        GPIO.output(DEFAULT_DIR_PIN, BACKWARD_LEVEL)
        pwm_output.start(speed)

    def stop(self):
        pwm_output.stop()
