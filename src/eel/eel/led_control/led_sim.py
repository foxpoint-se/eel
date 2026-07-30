from typing import Protocol


class LEDBackend(Protocol):
    def sequence(self, nof_pulses: int = 3, pulse_time: float = 0.1) -> None:
        pass

    def on(self) -> None:
        pass

    def off(self) -> None:
        pass


class LEDSimulator:
    def sequence(self, nof_pulses: int = 3, pulse_time: float = 0.1) -> None:
        pass

    def on(self) -> None:
        pass

    def off(self) -> None:
        pass
