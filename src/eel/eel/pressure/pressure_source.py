from abc import abstractmethod


class PressureSource:
    @abstractmethod
    def get_current_depth(self) -> float | None:
        pass
