from abc import abstractmethod, ABC
from typing import Literal, Optional

Direction = Literal["emptying", "filling"]


class Tank(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.current_direction: Optional[Direction] = None

    @property
    def is_emptying(self) -> bool:
        return self.current_direction == "emptying"

    @property
    def is_filling(self) -> bool:
        return self.current_direction == "filling"

    def empty(self) -> None:
        self.current_direction = "emptying"

    def fill(self) -> None:
        self.current_direction = "filling"

    def stop(self) -> None:
        self.current_direction = None

    def shutdown(self) -> None:
        self.stop()

    @abstractmethod
    def get_level(self) -> float:
        pass
