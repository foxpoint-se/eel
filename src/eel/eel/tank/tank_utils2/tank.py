from .interfaces import Pump, Sensor


class Tank:
    def __init__(self, pump: Pump, sensor: Sensor):
        self._pump = pump
        self._sensor = sensor

    def set_speed(self, speed: float):
        if abs(speed) < 0.01:
            self.stop()
            return

        self._pump.set_speed(speed)

    def stop(self):
        self._pump.stop()

    def get_level(self) -> float:
        return self._sensor.get_level()
