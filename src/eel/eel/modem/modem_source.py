from abc import abstractmethod
from typing import Union


class ModemSource:
    @abstractmethod
    def get_registration_status(self) -> Union[int, None]:
        pass
    
    @abstractmethod
    def get_received_signal_strength_indicator(self) -> Union[int, None]:
        pass
