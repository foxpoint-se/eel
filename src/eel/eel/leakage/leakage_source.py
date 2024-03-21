from abc import abstractmethod


class LeakageSource:
    @abstractmethod
    def read_sensor(self):
        pass