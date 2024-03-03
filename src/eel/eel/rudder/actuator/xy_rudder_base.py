from abc import abstractmethod, ABC

from .types import Vector2d


class XYRudder(ABC):
    @abstractmethod
    def set_rudder(self, value: Vector2d) -> None:
        pass

    @abstractmethod
    def shutdown(self) -> None:
        pass
