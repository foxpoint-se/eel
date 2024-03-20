import RPi.GPIO as GPIO

from .leakage_source import LeakageSource

LEAKAGE_LEVEL = GPIO.HIGH
NO_LEAKAGE_LEVEL = GPIO.LOW


# Note on the leakage sensor: Might be that we have several of these sensor, then 
# we should enable to provide an array with input pins, and this sensor will read all and act as one


class LeakageSensor(LeakageSource):
    def __init__(self, sensor_pin=15) -> None:
        self.sensor_pin=sensor_pin #TODO Check with Gunnar what is the correct PIN
        GPIO.setup(self.sensor_pin, GPIO.IN)
    
    def read_sensor(self):
        value = GPIO.input(self.sensor_pin)
        return_value = True if value == LEAKAGE_LEVEL else False

        return return_value


if __name__ == "__main__":
    sensor = LeakageSensor()
    leakage_value = sensor.read_sensor()

    print(f"Reading the sensor returned value {leakage_value}!")
