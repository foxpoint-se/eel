from abc import abstractmethod


class DistanceSensorBase:
    @abstractmethod
    def get_level(self) -> float:
        pass
