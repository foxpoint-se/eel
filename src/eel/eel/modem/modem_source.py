from abc import abstractmethod


class ModemSource:
    @abstractmethod
    def get_registration_status(self) -> int | None:
        pass

    @abstractmethod
    def get_received_signal_strength_indicator(self) -> int | None:
        pass

    @abstractmethod
    def ping(self) -> bool:
        pass
