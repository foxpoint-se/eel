from time import time, sleep


def calculate_position_delta(linear_velocity, time_delta):
    return linear_velocity * time_delta


class DistanceSensorSimulator:
    def __init__(
        self,
        initial_measurement_mm=None,
        update_frequency_hz=None,
        fill_velocity_mmps=None,
        create_timer=None,
        get_is_motor_filling_up=None,
        get_is_motor_emptying=None,
    ):
        if None in [
            initial_measurement_mm,
            update_frequency_hz,
            fill_velocity_mmps,
            create_timer,
            get_is_motor_filling_up,
            get_is_motor_emptying,
        ]:
            raise TypeError("all arguments to distance sensor simulator are required")

        self.last_updated_at = time()
        self.current_range = initial_measurement_mm
        self.fill_velocity_mmps = fill_velocity_mmps
        self.get_is_motor_filling_up = get_is_motor_filling_up
        self.get_is_motor_emptying = get_is_motor_emptying
        self.distance_updater = create_timer(
            1.0 / (update_frequency_hz), self._update_range
        )

    def get_range(self):
        # to simulate that actual sensor measurement takes some time, depending on `timing_budget`
        sleep(0.3)
        return self.current_range

    def _update_range(self):
        now = time()
        time_delta = now - self.last_updated_at
        position_delta = calculate_position_delta(self.fill_velocity_mmps, time_delta)
        if self.get_is_motor_filling_up():
            self.current_range = self.current_range + position_delta
        elif self.get_is_motor_emptying():
            self.current_range = self.current_range - position_delta

        self.last_updated_at = time()
