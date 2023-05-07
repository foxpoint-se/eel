from time import time


class PidInterface:
    def compute(self, system_current_value: float) -> float:
        """Compute next control signal, given current system value."""
        pass

    def update_target(self, set_point: float = None) -> None:
        """Update target (set point)."""
        pass


class PidController(PidInterface):
    def __init__(
        self, set_point, kP=0.0, kI=0.0, kD=0.0, on_log_error=None
    ) -> None:
        super().__init__()
        self.kP = kP  # proportional gain
        self.kI = kI  # integral gain
        self.kD = kD  # derivative gain
        self.set_point = set_point
        self.last_computed_at = None
        self.cumulative_error = 0.0
        self.last_error = 0.0
        self.on_log_error = on_log_error

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
        return p + i + d

    def update_target(self, set_point: float = None) -> None:
        self.set_point = set_point


depth_pid_1 = PidController(set_point=None, kP=0.25, kI=0.0, kD=0.0)


pitch_pid_1 = PidController(set_point=None, kP=0.02, kI=0.0, kD=0.127)
