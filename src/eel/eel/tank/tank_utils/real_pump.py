import RPi.GPIO as GPIO

FILL_GPIO_LEVEL = GPIO.HIGH
EMPTY_GPIO_LEVEL = GPIO.LOW

RUN_GPIO_LEVEL = GPIO.HIGH
STOP_GPIO_LEVEL = GPIO.LOW

# PWM_FREQUENCY = 4096
PWM_FREQUENCY = 19200


# TODO:
# se om jag kan få tankarna att funka på tvålen igen
# se om jag fattar varför vi har simulerat saker på imu
# att den lutar olika. det är väl för att tankarna inte är centrerade

# TODO: kolla på varningen om lgpio
# gpiozero==2.0 kanske? googla på felet


class RealPump:
    def __init__(self, motor_pin: int, direction_pin: int) -> None:
        self.motor_pin = motor_pin
        self.direction_pin = direction_pin

        # Initialize PI pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.motor_pin, GPIO.OUT)

        self.pwm_output = GPIO.PWM(self.motor_pin, PWM_FREQUENCY)

    def _set_filling_up(self):
        GPIO.output(self.direction_pin, FILL_GPIO_LEVEL)

    def _set_emptying(self):
        GPIO.output(self.direction_pin, EMPTY_GPIO_LEVEL)

    def _start_motor(self):
        # GPIO.output(self.motor_pin, RUN_GPIO_LEVEL)
        # self.pwm_output.start(80)
        self.pwm_output.start(60)

    def _stop_motor(self):
        # GPIO.output(self.motor_pin, STOP_GPIO_LEVEL)
        self.pwm_output.stop()

    def stop(self):
        self._stop_motor()

    def fill(self):
        self._set_filling_up()
        self._start_motor()

    def empty(self):
        self._set_emptying()
        self._start_motor()
