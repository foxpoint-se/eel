from time import time

VELOCITY_IN_MM_PER_SECOND = 3


def calculate_position_delta(linear_velocity, time_delta):
    return linear_velocity * time_delta


class DistanceSensorSimulator:
    def __init__(self, parent_node=None):
        self.last_updated_at = time()
        self.parent_node = parent_node
        self.current_range = parent_node.RANGE_FLOOR
        self.distance_updater = parent_node.create_timer(
            1.0 / (parent_node.update_frequency * 2), self._update_range
        )

    def get_range(self):
        return self.current_range

    def _update_range(self):
        if self.parent_node.pump_motor_control.get_is_on():
            is_forward = self.parent_node.pump_motor_control.get_is_forward()
            now = time()
            time_delta = now - self.last_updated_at
            position_delta = calculate_position_delta(
                VELOCITY_IN_MM_PER_SECOND, time_delta
            )
            if is_forward:
                self.current_range = self.current_range + position_delta
            else:
                self.current_range = self.current_range - position_delta

        self.last_updated_at = time()
