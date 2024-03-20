import time

from .leakage_source import LeakageSource


SIMULATED_LEAKAGE_TIME_SECONDS = 120


class LeakageSimulator(LeakageSource):
    def __init__(self) -> None:
        self.start_time = time.time()

    def read_sensor(self) -> bool | None:
        return_value = False
        
        # If more than X amount of seconds passed, start sending True
        if time.time() - self.start_time > SIMULATED_LEAKAGE_TIME_SECONDS:
            return_value = True

        return return_value
