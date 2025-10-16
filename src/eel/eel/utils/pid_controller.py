from time import time


class PidController:
    def __init__(
        self,
        set_point,
        kP=0.0,
        kI=0.0,
        kD=0.0,
        output_min=-float("inf"),
        output_max=float("inf"),
    ) -> None:
        self.kP = kP  # proportional gain
        self.kI = kI  # integral gain
        self.kD = kD  # derivative gain
        self.output_min = output_min
        self.output_max = output_max
        self.set_point = set_point
        self.last_computed_at = None
        self.cumulative_error = 0.0
        self.last_error = 0.0
        self.on_log_error = on_log_error

    def update_set_point(self, value):
        self.set_point = value
 
    def reset_cumulative_error(self):
        self.cumulative_error = 0

    def compute(self, system_current_value):
        now = time()

        if self.last_computed_at is None:
            self.last_computed_at = now

        error = self.set_point - system_current_value

        if self.on_log_error:
            self.on_log_error(error)

        p = self.kP * error

        time_delta = now - self.last_computed_at
        self.cumulative_error += error * time_delta

        i = self.cumulative_error * self.kI

        error_delta = error - self.last_error

        rate_of_error = 0.0

        if time_delta > 0:
            rate_of_error = error_delta / time_delta

        d = rate_of_error * self.kD

        self.last_error = error

        self.last_computed_at = now

        output = p + i + d

        # Clamp final output
        return max(self.output_min, min(self.output_max, output))
