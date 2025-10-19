from .interfaces import Pump, Sensor


from time import time


class SimState:
    def __init__(self, initial_level: float):
        self.level = initial_level
        self.pump_speed = 0.0
        self.last_update = time()


class SimPump(Pump):
    def __init__(self, state: SimState):
        self.state = state

    def set_speed(self, speed: float):
        self.state.pump_speed = speed

    def stop(self):
        self.state.pump_speed = 0.0


class SimSensor(Sensor):
    def __init__(self, state: SimState):
        self.state = state

    def get_level(self) -> float:
        self._update_simulation()
        return self.state.level

    def _update_simulation(self):
        now = time()
        dt = now - self.state.last_update
        if self.state.pump_speed > 0:
            velocity = 0.05  # Fill rate
        elif self.state.pump_speed < 0:
            velocity = -0.03  # Empty rate
        else:
            velocity = 0

        self.state.level += velocity * dt
        self.state.level = max(0, min(1, self.state.level))
        self.state.last_update = now
