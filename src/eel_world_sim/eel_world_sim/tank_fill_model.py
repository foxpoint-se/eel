"""Tank fill physics for the world plant. No ROS — pump cmd in, level out.

Rates match the old SimTank stub (percent of full per second).
"""

FILL_VELOCITY_PER_S = 0.05
EMPTY_VELOCITY_PER_S = 0.03
MIN_LEVEL = 0.0
MAX_LEVEL = 1.0


def _cap_level(level: float) -> float:
    return min(max(level, MIN_LEVEL), MAX_LEVEL)


class TankFillModel:
    """Integrates tank level from pump command over time."""

    def __init__(self) -> None:
        self.level = 0.0
        self.pump_cmd = 0.0

    def set_pump_cmd(self, pump_cmd: float) -> None:
        self.pump_cmd = pump_cmd

    def step(self, dt_s: float) -> float:
        if dt_s <= 0.0:
            return self.level

        if self.pump_cmd > 0.0:
            self.level = _cap_level(self.level + FILL_VELOCITY_PER_S * dt_s)
        elif self.pump_cmd < 0.0:
            self.level = _cap_level(self.level - EMPTY_VELOCITY_PER_S * dt_s)

        return self.level
