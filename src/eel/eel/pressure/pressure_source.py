from abc import abstractmethod
from typing import Union


class PressureSource:
    @abstractmethod
    def get_current_depth(self) -> Union[float, None]:
        pass
