from abc import ABC, abstractmethod


class Pump(ABC):
    @abstractmethod
    def set_speed(self, speed: float):
        pass

    @abstractmethod
    def stop(self):
        pass


class Sensor(ABC):
    @abstractmethod
    def get_level(self) -> float:
        pass
