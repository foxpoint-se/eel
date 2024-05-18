import RPi.GPIO as GPIO

from .leakage_source import LeakageSource

LEAKAGE_LEVEL = GPIO.HIGH
NO_LEAKAGE_LEVEL = GPIO.LOW


class LeakageSensor(LeakageSource):
    def __init__(self, sensor_pin=17) -> None:
        self.sensor_pin=sensor_pin
        GPIO.setup(self.sensor_pin, GPIO.IN)
    
    def read_sensor(self):
        value = GPIO.input(self.sensor_pin)
        return_value = True if value == LEAKAGE_LEVEL else False

        return return_value


if __name__ == "__main__":
    sensor = LeakageSensor()
    leakage_value = sensor.read_sensor()

    print(f"Reading the sensor returned value {leakage_value}!")
