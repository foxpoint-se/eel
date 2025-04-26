from abc import abstractmethod, ABC

from .types import Vector2d


class XYRudder(ABC):
    @abstractmethod
    def set_rudder(self, value: Vector2d) -> None:
        pass
    
    @abstractmethod
    def get_x_rudder_offset_value(self) -> float:
        pass
    
    @abstractmethod
    def get_y_rudder_offset_value(self) -> float:
        pass
    
    @abstractmethod
    def get_x_rudder_cap_max_value(self) -> float:
        pass

    @abstractmethod
    def get_x_rudder_cap_min_value(self) -> float:
        pass
    
    @abstractmethod
    def get_y_rudder_cap_max_value(self) -> float:
        pass
    
    @abstractmethod
    def get_y_rudder_cap_min_value(self) -> float:
        pass

    @abstractmethod
    def set_x_rudder_offset_value(self, value: float) -> None:
        pass
    
    @abstractmethod
    def set_y_rudder_offset_value(self, value: float) -> None:
        pass

    @abstractmethod
    def shutdown(self) -> None:
        pass
