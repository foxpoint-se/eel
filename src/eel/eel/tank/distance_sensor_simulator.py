from time import time
from .distance_sensor_base import DistanceSensorBase


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


class DistanceSensorSimulator(DistanceSensorBase):
    def __init__(
        self,
        initial_measurement_percent=None,
        update_frequency_hz=None,
        create_timer=None,
        get_is_motor_filling_up=None,
        get_is_motor_emptying=None,
    ):
        if None in [
            initial_measurement_percent,
            update_frequency_hz,
            create_timer,
            get_is_motor_filling_up,
            get_is_motor_emptying,
        ]:
            raise TypeError("all arguments to distance sensor simulator are required")

        self.last_updated_at = time()
        self.current_level = initial_measurement_percent
        self.get_is_motor_filling_up = get_is_motor_filling_up
        self.get_is_motor_emptying = get_is_motor_emptying
        self.distance_updater = create_timer(
            1.0 / (update_frequency_hz), self._update_level
        )

    def _update_level(self):
        now = time()
        time_delta = now - self.last_updated_at
        if self.get_is_motor_filling_up():
            position_delta = calculate_position_delta(
                FILL_VELOCITY_PERCENT_PER_SECOND, time_delta
            )
            self.current_level = self.current_level - position_delta
        elif self.get_is_motor_emptying():
            position_delta = calculate_position_delta(
                EMPTY_VELOCITY_PERCENT_PER_SECOND, time_delta
            )
            self.current_level = self.current_level + position_delta

        self.current_level = cap_value(self.current_level, 0.0, 1.0)
        self.last_updated_at = time()

    def get_level(self) -> float:
        return float(self.current_level)
