from time import time
from typing import Optional


def _clamp(x, lo, hi):
    if lo is None and hi is None:
        return x
    if lo is None:
        return min(x, hi)
    if hi is None:
        return max(x, lo)
    return max(lo, min(hi, x))


class PidController:
    def __init__(
        self,
        set_point: float,
        kP: float = 0.0,
        kI: float = 0.0,
        kD: float = 0.0,
        output_min: float = -float("inf"),
        output_max: float = float("inf"),
        integrator_min: Optional[float] = None,
        integrator_max: Optional[float] = None,
    ) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.output_min = output_min
        self.output_max = output_max
        self.set_point = set_point

        self.last_computed_at = None
        self.cumulative_error = 0.0  # integral accumulator (error * dt)
        self.last_error = 0.0

        # integrator limits are bounds on the *i term* (kI * cumulative_error)
        self.integrator_min = integrator_min
        self.integrator_max = integrator_max

    def update_set_point(self, value: float) -> None:
        self.set_point = value

        # Resetting cumulative error, to avoid windup when set point changes
        self.cumulative_error = 0.0

    def reset_cumulative_error(self) -> None:
        self.cumulative_error = 0.0

    def compute(self, system_current_value: float) -> float:
        now = time()
        if self.last_computed_at is None:
            self.last_computed_at = now

        error = self.set_point - system_current_value
        p = self.kP * error

        time_delta = now - self.last_computed_at
        # integrate
        self.cumulative_error += error * time_delta

        # --- Anti-windup: limit the integral term -----------------------------------
        # The cumulative_error is stored in "error × time" units, but what really matters
        # for the controller output is (kI * cumulative_error) — the i-term itself.
        # To make the clamp operate in output units, we divide the desired i-term limits
        # by kI before applying them to cumulative_error.
        #
        # Example:
        #   kI = 0.02, integrator_max = 0.4  ->  cumulative_error limited to 0.4 / 0.02 = 20
        #   So i = kI * cumulative_error  ->  max i-term = ±0.4
        #
        # Symmetric ± limits allow the integrator to both increase and decrease the output,
        # preventing steady-state bias and avoiding integrator windup.
        if self.kI != 0.0 and (
            self.integrator_min is not None or self.integrator_max is not None
        ):
            # compute allowed cumulative_error bounds so that i = kI * cumulative_err is within integrator_min/max
            i_min = (
                None if self.integrator_min is None else (self.integrator_min / self.kI)
            )
            i_max = (
                None if self.integrator_max is None else (self.integrator_max / self.kI)
            )
            self.cumulative_error = _clamp(self.cumulative_error, i_min, i_max)

        i = self.cumulative_error * self.kI

        # derivative
        error_delta = error - self.last_error
        rate_of_error = 0.0
        if time_delta > 0:
            rate_of_error = error_delta / time_delta
        d = rate_of_error * self.kD

        self.last_error = error
        self.last_computed_at = now

        output = p + i + d
        # clamp output
        output = max(self.output_min, min(self.output_max, output))
        return output
