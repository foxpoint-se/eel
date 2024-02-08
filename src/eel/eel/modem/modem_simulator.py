from typing import Union
from .modem_source import ModemSource

class ModemSimulator(ModemSource):
    def get_received_signal_strength_indicator(self) -> Union[int, None]:
        return 31
    
    def get_registration_status(self) -> Union[int, None]:
        return 1
