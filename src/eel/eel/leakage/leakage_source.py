from abc import abstractmethod
from typing import Union


class LeakageSource:
    @abstractmethod
    def read_sensor(self) -> Union[bool, None]:
        pass