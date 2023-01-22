from .distance_sensor_base import DistanceSensorBase


class DistanceSensorPotentiometer(DistanceSensorBase):
    def __init__(self) -> None:
        super().__init__()

    def get_range(self) -> float:
        return 33
