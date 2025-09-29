from time import time
from rclpy import logging
from .tank_base import Tank

ros_logger = logging.get_logger(__name__)


def calculate_position_delta(linear_velocity, time_delta):
    return linear_velocity * time_delta


def cap_value(value, floor, ceiling):
    if value > ceiling:
        return ceiling
    elif value < floor:
        return floor
    return value


FILL_VELOCITY_PERCENT_PER_SECOND = 0.05
EMPTY_VELOCITY_PERCENT_PER_SECOND = 0.03


class SimTank(Tank):
    def __init__(self) -> None:
        super().__init__()
        self.current_level: float = float()
        self.last_updated_at = time()
        ros_logger.info("Started sim tank.")

    def get_level(self) -> float:
        self._update_level()
        return self.current_level

    def _update_level(self) -> None:
        now = time()
        time_delta = now - self.last_updated_at
        if self.current_direction:
            if self.current_direction == "filling":
                position_delta = calculate_position_delta(
                    FILL_VELOCITY_PERCENT_PER_SECOND, time_delta
                )
                next_level = self.current_level + position_delta
            else:
                position_delta = calculate_position_delta(
                    EMPTY_VELOCITY_PERCENT_PER_SECOND, time_delta
                )
                next_level = self.current_level - position_delta

            self.current_level = cap_value(next_level, 0.0, 1.0)

        self.last_updated_at = time()

    def run_motor(self, value) -> None:
        return super().run_motor(value)
