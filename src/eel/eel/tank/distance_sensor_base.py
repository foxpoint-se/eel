from abc import abstractmethod


class DistanceSensorBase:
    @abstractmethod
    def get_range(self) -> float:
        pass
