#!/usr/bin/python3
from .interfaces import Pump, Sensor
import RPi.GPIO as GPIO
from gpiozero import MCP3208

FILL_GPIO_LEVEL = GPIO.HIGH
EMPTY_GPIO_LEVEL = GPIO.LOW

RUN_GPIO_LEVEL = GPIO.HIGH
STOP_GPIO_LEVEL = GPIO.LOW

# PWM_FREQUENCY = 4096
PWM_FREQUENCY = 19200


class HardwarePump(Pump):
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

    def _run_motor(self, pwm_value):
        self.pwm_output.start(pwm_value)

    def _stop_motor(self):
        self.pwm_output.stop()

    def stop(self):
        self._stop_motor()

    def set_speed(self, speed: float):
        pwm_value = speed * 80.0
        if pwm_value > 0:
            self._set_filling_up()
        else:
            self._set_emptying()
        self._run_motor(abs(pwm_value))


def translate_from_range_to_range(value, from_min, from_max, to_min, to_max):
    # Figure out how 'wide' each range is
    left_span = from_max - from_min
    right_span = to_max - to_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - from_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return to_min + (value_scaled * right_span)


def cap_value(value, floor, ceiling):
    if value > max([ceiling, floor]):
        return max([ceiling, floor])
    elif value < min([ceiling, floor]):
        return min([ceiling, floor])
    return value


class HardwareSensor(Sensor):
    def __init__(self, floor: float, ceiling: float, channel: int):
        super().__init__()
        self.channel = channel
        self.floor = floor
        self.ceiling = ceiling

        self.mcp = MCP3208(
            channel=self.channel,
            differential=False,
            max_voltage=3.3,
            clock_pin=11,
            mosi_pin=10,
            miso_pin=9,
            select_pin=8,
        )

    def _get_raw_value(self) -> float:
        return self.mcp.value

    def _get_pretty_value(self) -> float:
        raw = self._get_raw_value()
        capped = cap_value(raw, self.floor, self.ceiling)
        level = translate_from_range_to_range(
            capped,
            self.floor,
            self.ceiling,
            0.0,
            1.0,
        )

        return level

    def get_level(self) -> float:
        return self._get_pretty_value()
