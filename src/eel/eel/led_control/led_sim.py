from typing import Protocol


class LEDBackend(Protocol):
    def on(self) -> None:
        pass

    def off(self) -> None:
        pass


class LEDSimulator:
    def on(self) -> None:
        pass

    def off(self) -> None:
        pass
